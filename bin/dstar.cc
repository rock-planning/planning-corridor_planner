#include <gdal.h>
#include <gdal_priv.h>
#include <dfki/base_types.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <boost/tuple/tuple.hpp>

#include <nav/dstar.hh>
#include <nav/skeleton.hh>
#include <nav/merge.hh>

using namespace std;
using DFKI::Time;
using namespace boost;
using namespace Nav;

struct RGBColor
{
    uint8_t r, g, b;
    RGBColor()
        : r(0), g(0), b(0) {}
    explicit RGBColor(uint8_t grey)
        : r(grey), g(grey), b(grey) {}
    explicit RGBColor(uint8_t r, uint8_t g, uint8_t b)
        : r(r), g(g), b(b) {}
};

void saveColorImage(string const& path, int xSize, int ySize, vector<RGBColor>& pixels)
{
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Gtiff");
    auto_ptr<GDALDataset> output_set(driver->Create(path.c_str(), xSize, ySize, 3, GDT_Byte, 0));

    output_set->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &pixels[0].r, xSize, ySize, GDT_Byte, sizeof(RGBColor), sizeof(RGBColor) * xSize);
    output_set->GetRasterBand(2)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &pixels[0].g, xSize, ySize, GDT_Byte, sizeof(RGBColor), sizeof(RGBColor) * xSize);
    output_set->GetRasterBand(3)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &pixels[0].b, xSize, ySize, GDT_Byte, sizeof(RGBColor), sizeof(RGBColor) * xSize);
}

template<typename Collection>
void markPoints(Collection const& points, int xSize, vector<RGBColor> & pixels, RGBColor color)
{
    for (typename Collection::const_iterator point_it = points.begin(); point_it != points.end(); ++point_it)
    {
        int x = point_it->x, y = point_it->y;
        int idx = x + y * xSize;

        pixels[idx] = color;
    }
}

vector<RGBColor> allocateColors(size_t count)
{
    vector<RGBColor> result;
    if (count == 1)
        result.push_back(RGBColor(100, 100, 255));
    else if (count == 2)
    {
        result.push_back(RGBColor(100, 100, 255));
        result.push_back(RGBColor(255, 100, 100));
    }
    else if (count > 2)
    {
        float step = 155.0 / (count / 2);
        for (size_t i = 0; i < count / 2; ++i)
            result.push_back(RGBColor(100, 100 + step * i, 255 - step * i));

        step = 155.0 / (count - count / 2);
        for (size_t i = 0; i < count - count / 2; ++i)
            result.push_back(RGBColor(100 + step * i, 255 - step * i, 100 + step * i));
    }

    random_shuffle(result.begin(), result.end());
    return result;
}

struct Profile
{
    char const* desc;
    Time start_time;

    Profile(char const* desc)
        : desc(desc)
        , start_time(Time::now())
    {
        cerr << "starting " << desc << endl;
    }
    ~Profile()
    {
        cerr << desc << ": " << (Time::now() - start_time).toMilliseconds() << endl;
    }
};

void outputPlan(int xSize, int ySize, std::string const& basename, std::vector<uint8_t> const& image, Plan const& plan)
{
    vector<RGBColor> color_image;
    // Initialize with the real image
    for (size_t i = 0; i < image.size(); ++i)
        color_image.push_back(RGBColor(image[i]));
    // Get a color set for the corridors
    vector<RGBColor> colors = allocateColors(plan.corridors.size());

    for (size_t corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
    {
        Corridor const& c = plan.corridors[corridor_idx];
        MedianPoint::BorderList::const_iterator border_it;
        for (border_it = c.borders.begin(); border_it != c.borders.end(); ++border_it)
            markPoints(*border_it, xSize, color_image, colors[corridor_idx]);

        MedianLine::const_iterator median_it;
        PointSet median_points;
        for (median_it = c.median.begin(); median_it != c.median.end(); ++median_it)
            median_points.insert(median_it->center);
        markPoints(median_points, xSize, color_image, colors[corridor_idx]);
    }

    string corridor_out = basename + "-corridors.tif";
    cerr << "  saving image in " << corridor_out << endl;
    saveColorImage(corridor_out, xSize, ySize, color_image);
    string dot_out = basename + "-corridors.dot";
    cerr << "  saving result in " << dot_out << endl;
    ofstream dot(dot_out.c_str());
    dot << "digraph {\n";
    for (size_t corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
    {
        RGBColor color = colors[corridor_idx];
        dot << "  c" << corridor_idx << "[color=\"#" << hex
            << (int)color.r << (int)color.g << (int)color.b << "\"];\n" << dec;

        std::set<int> seen;
        Corridor::Connections const& connections = plan.corridors[corridor_idx].connections;
        Corridor::Connections::const_iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {

            int target_idx = conn_it->get<1>();
            if (!seen.count(target_idx))
            {
                dot << "  c" << corridor_idx << " -> c" << target_idx << ";\n";
                seen.insert(target_idx);
            }
        }
    }
    dot << "}\n";
}


tuple<Plan, uint32_t, uint32_t, vector<uint8_t> > do_terrain(std::string const& basename, std::string const& terrain, TerrainClasses const& terrain_classes,
        int x0, int y0, int x1, int y1, float expand)
{
    // Load the file and run D*
    TraversabilityMap* map = TraversabilityMap::load(terrain, terrain_classes);
    if (!map)
        throw std::runtime_error("cannot load file");

    DStar algo(*map, terrain_classes);
    GridGraph const& graph = algo.graph();
    uint32_t const xSize = graph.xSize(), ySize = graph.ySize();

    // Load the original terrain file, we will superimpose some data later on
    // it.
    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(terrain.c_str(), GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);
    vector<uint8_t> image(xSize * ySize);
    band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);

    std::cerr << "applying D* on a " << graph.xSize() << "x" << graph.ySize() << " map, target point is " << x1 << "x" << y1 << std::endl;
    { Profile profiler("dstar");
        algo.initialize(x1, y1);
    }
    std::cerr << "  done ... checking solution consistency" << std::endl;
    algo.checkSolutionConsistency();

    {
        string dstar_out = basename + "-dstar.tif";
        std::cerr << "  saving result in " << dstar_out << std::endl;

        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Gtiff");
        auto_ptr<GDALDataset> output_set(driver->Create(dstar_out.c_str(), xSize, ySize, 1, GDT_Float32, 0));

        vector<float> costs;
        costs.reserve(xSize * ySize);
        for (uint32_t y = 0; y < ySize; ++y)
            for (uint32_t x = 0; x < xSize; ++x)
                costs.push_back(graph.getValue(x, y));

        output_set->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize, ySize,
                &costs[0], xSize, ySize, GDT_Float32, 0, 0);
    }
    
    MedianLine result;
    SkeletonExtraction skel(xSize, ySize);
    { Profile profiler("computing skeleton");
        result = skel.processDStar(algo, x0, y0, expand);
    }

    { 
        pair<PointSet, PointSet> border = skel.getBorderAndInside();
        vector<RGBColor> color_image;
        for (size_t i = 0; i < image.size(); ++i)
            color_image.push_back(RGBColor(image[i]));

        markPoints(border.first, xSize, color_image, RGBColor(128, 128, 255));
        string out = basename + "-border.tif";
        std::cerr << "  saving result in " << out << std::endl;
        saveColorImage(out, xSize, ySize, color_image);
    }

    Plan plan(PointID(x0, y0), PointID(x1, y1), graph);
    { Profile profiler("computing plan");
        skel.buildPlan(plan, result);
    }

    cerr << plan.corridors.size() << " corridors found" << endl;

    outputPlan(xSize, ySize, basename, image, plan);
    return make_tuple(plan, xSize, ySize, image);
}

int main(int argc, char** argv)
{
    if (argc != 6 && argc != 9)
    {
	std::cerr 
	    << "usage: dstar terrain_file terrain_classes x1 y1 basename [x0 y0 expand_factor]\n"
            << "  computes the result of D* on the given map, with [x1, y1] as\n"
            << "  the goal. If x0, y0 and expand_factor are given, then we also\n"
            << "  compute the expanded zone of navigation around the optimal path\n"
            << "  starting at x0 y0\n"
            << "\n"
            << "  the result is saved into two multiple files.\n"
            << "  <basename>-dstar.txt is the output of DStar in the following format:\n"
            << "\n"
            << "  width height\n"
            << "  x0 y0 x1 y1\n"
            << "  x y cost parentX parentY\n"
            << "  x y cost parentX parentY\n"
            << "  ...\n"
            << "\n"
            << "  where (x, y) is the cell coordinates, 'cost' the cost of the\n"
            << "  path to the goal and (parentX, parentY) the coordinates of\n"
            << "  the next cell in the path to the goal\n"
            << "\n"
            << "  <basename>-border.txt is the outer part of the border of the\n"
            << "  filtered output\n"
            << "\n"
            << "  <basename>-skeleton.txt is the descriptor of the generated\n"
            << "  corridors\n"
            << std::endl;
	exit(1);

    }
    GDALAllRegister();

    string terrain_file    = argv[1];
    string terrain_classes = argv[2];
    int x1 = boost::lexical_cast<int>(argv[3]);
    int y1 = boost::lexical_cast<int>(argv[4]);
    std::string out_basename = argv[5];
 
    // Load the terrain classes
    TerrainClasses classes = TerrainClass::load(terrain_classes);

    int x0 = boost::lexical_cast<int>(argv[6]);
    int y0 = boost::lexical_cast<int>(argv[7]);
    float expand = boost::lexical_cast<float>(argv[8]);

    int xSize, ySize; vector<uint8_t> image;
    Plan original;
    tie(original, xSize, ySize, image) = do_terrain(out_basename, terrain_file + ".tif", classes, x0, y0, x1, y1, expand);
    Plan blocked = do_terrain(out_basename + "-blocked", terrain_file + "-blocked.tif", classes, x0, y0, x1, y1, expand).get<0>();

    //int original_idx = original.findEndpointCorridor( PointID(76, 140) );
    //int blocked_idx  = blocked.findEndpointCorridor( PointID(76, 140) );
    //for (int i = 0; i < original.corridors.size(); ++i)
    //{
    //    for (int j = 0; j < blocked.corridors.size(); ++j)
    //    {
            //int i = 4;
            //int j = 43;
            //PlanMerge merger;
            //Plan left, right;
            //left.corridors.push_back(original.corridors[i]);
            //left.corridors[0].connections.clear();
            //right.corridors.push_back(blocked.corridors[4]);
            //right.corridors[0].connections.clear();

            //merger.merge(left, right, 0.5, 0.45);
    //    }
    //}
    PlanMerge merger;
    merger.merge(original, blocked, 0.5, 0.1);
    cerr << merger.corridors.size() << " corridors in merged plan" << endl;
    outputPlan(xSize, ySize, out_basename + "-merge", image, merger);
    return 0;
}

