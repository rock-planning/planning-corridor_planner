#include <gdal.h>
#include <gdal_priv.h>
#include <base/time.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <boost/tuple/tuple.hpp>

#include "dstar.hh"
#include "skeleton.hh"
#include "merge.hh"
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

using namespace std;
using base::Time;
using namespace boost;
using namespace nav;

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

void markCurve(geometry::NURBSCurve3D& curve, int xSize, vector<RGBColor> & pixels, RGBColor color)
{
    double t       = curve.getStartParam();
    double unit_t  = curve.getUnitParameter();
    double end_t   = curve.getEndParam();

    for (; t <= end_t; t += unit_t / 4)
    {
        Eigen::Vector3d p = curve.getPoint(t);
        int x = lround(p.x()), y = lround(p.y());
        int idx = x + xSize * y;
        pixels[idx] = color;
    }
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
    else if (count < 4)
    {
        result.push_back(RGBColor(100, 100, 255));
        result.push_back(RGBColor(255, 100, 100));
    }
    else if (count > 2)
    {
        // result:
        //   100, 100, 255
        //   100, 255, 100
        //   255, 100, 100
        //   100, 255, 255
        //   255, 100, 255

        size_t   gradient_size = (count + 5) / 6;
        float step = 155.0 / gradient_size;
        for (size_t i = 0; i < gradient_size; ++i)
        {
            result.push_back(RGBColor(100, 100 + step * i, 255 - step * i));
            result.push_back(RGBColor(100 + step * i, 255 - step * i, 100));
            result.push_back(RGBColor(255 - step * i, 100 + step * i, 100 + step * i));

            result.push_back(RGBColor(0, step * i, 155 - step * i));
            result.push_back(RGBColor(step * i, 155 - step * i, 0));
            result.push_back(RGBColor(155 - step * i, step * i, step * i));
        }
    }

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

string colorToDot(RGBColor const& color)
{
    ostringstream stream;

    stream << "#" << hex;

#define DISPLAY_COLOR_CODE(io, code) \
        if ((code) < 16) \
            io << 0; \
        io << (int)(code);

    DISPLAY_COLOR_CODE(stream, color.r);
    DISPLAY_COLOR_CODE(stream, color.g);
    DISPLAY_COLOR_CODE(stream, color.b);
    return stream.str();
}

void outputPlan(int xSize, int ySize, std::string const& basename, std::vector<uint8_t> const& image, Plan& plan)
{
    vector<RGBColor> color_image;
    // Initialize with the real image
    for (size_t i = 0; i < image.size(); ++i)
        color_image.push_back(RGBColor(image[i]));
    // Get a color set for the corridors
    vector<RGBColor> colors = allocateColors(plan.corridors.size());

    for (size_t corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
    {
        Corridor& c = plan.corridors[corridor_idx];

        markPoints(c.boundaries[0], xSize, color_image, colors[corridor_idx]);
        markPoints(c.boundaries[1], xSize, color_image, colors[corridor_idx]);

        vector<PointID> points;
        transform(c.voronoi.begin(), c.voronoi.end(),
                back_inserter(points), bind(&VoronoiPoint::center, _1));
        markPoints(points, xSize, color_image, colors[corridor_idx]);
    }

    string corridor_out = basename + "-corridors.tif";
    cerr << "  saving image in " << corridor_out << endl;
    saveColorImage(corridor_out, xSize, ySize, color_image);
    string dot_out = basename + "-corridors.dot";
    cerr << "  saving result in " << dot_out << endl;
    ofstream dot(dot_out.c_str());
    dot << "digraph {\n";
    dot << "  rankdir=LR;\n";
    for (size_t corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
    {
        RGBColor color = colors[corridor_idx];

        Corridor const& corridor = plan.corridors[corridor_idx];
        string corridor_name = corridor.name;

        if (corridor.isSingleton())
        {
            dot << "  c" << corridor_idx << "_0 "
                << "[label=\"" << corridor_name << "\", color=\""
                << colorToDot(color) << "\"]\n";
        }
        else
        {
            ostringstream node_setup;
            node_setup << "[fixedsize=true, width=0.1, height=0.1, label=\"\", style=filled, shape=circle, "
                << "color=\"" << colorToDot(color) << "\"];\n";

            dot << "  c" << corridor_idx << "_0 " << node_setup.str();
            dot << "  c" << corridor_idx << "_1 " << node_setup.str();
            dot << "  c" << corridor_idx << "_0 -> c" << corridor_idx
                << "_1 [label=\"" << corridor_name << "\", color=\""
                << colorToDot(color) << "\"]\n";

            if (corridor.bidirectional)
            {
                dot << "  c" << corridor_idx << "_1 -> c" << corridor_idx << "_0 [color=\""
                    << colorToDot(color) << "\"]\n";
            }
        }


        std::set<int> seen;
        if (corridor.voronoi.size() == 1)
            cerr << " corridor " << corridor.name << " is of size 1" << endl;

        Corridor::Connections const& connections = corridor.connections;
        Corridor::Connections::const_iterator conn_it;
        bool this_singleton = corridor.isSingleton();
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            int target_idx = conn_it->target_idx;
            bool target_singleton = plan.corridors[target_idx].isSingleton();

            if (!seen.count(target_idx))
            {
                int in_side  = conn_it->this_side;
                if (this_singleton)
                    in_side = false;
                int out_side = conn_it->target_side;
                if (target_singleton)
                    out_side = false;

                dot << "  c" << corridor_idx << "_" << in_side
                    << " -> c" << target_idx << "_" << out_side << " [weight=3];\n";
                seen.insert(target_idx);
            }
        }
    }
    dot << "}\n";
}

void outputExtractionState(int xSize, int ySize, std::string const& out, vector<uint8_t> const& image, CorridorExtractionState const& state)
{
    vector<Corridor> const& corridors = state.plan.corridors;

    cerr << "found " << corridors.size() << " corridors" << endl;
    vector<RGBColor> color_image;
    for (size_t i = 0; i < image.size(); ++i)
        color_image.push_back(RGBColor(image[i]));

    // Get colors. Note that we take two more colors to mark the connections
    vector<RGBColor> colors = allocateColors(corridors.size() + 2);

    // First mark the branches
    for (vector<Corridor>::const_iterator it = corridors.begin(); it != corridors.end(); ++it)
    {
        vector<PointID> points;
        list<VoronoiPoint> const& voronoi_points = it->voronoi;
        points.resize(voronoi_points.size());
        transform(voronoi_points.begin(), voronoi_points.end(), points.begin(),
                boost::bind(&VoronoiPoint::center, _1));

        markPoints(points, xSize, color_image, colors.back());
        colors.pop_back();
    }

    // Then mark the connection with the two leftover colors
    Crossroads const& connections = state.crossroads;
    SplitPoints const& splits = state.split_points;
    RGBColor connection_color = colors[0];
    RGBColor split_color = colors[1];
    for (Crossroads::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        vector<PointID> points;
        points.reserve(points.size() + it->size());
        for (list<Endpoint>::const_iterator endp_it = it->begin(); endp_it != it->end(); ++endp_it)
        {
            int target_idx   = endp_it->corridor_idx;
            bool target_side = endp_it->side;
            PointID p = state.plan.corridors[target_idx].getEndpoint(target_side);
            points.push_back(p);
        }

        //cerr << "connection points: ";
        //displayLine(cerr, points, std::_Identity<PointID>());
        SplitPoints::const_iterator split_it;
        for (split_it = splits.begin(); split_it != splits.end(); ++split_it)
        {
            if (split_it->first == it)
                break;
        }

        RGBColor color;
        if (split_it != splits.end())
            color = RGBColor(255, 200, 200);
        else
            color = RGBColor(255, 255, 255);
        markPoints(points, xSize, color_image, color);
    }

    std::cerr << "  saving result in " << out << std::endl;
    saveColorImage(out, xSize, ySize, color_image);
}


tuple<Plan, uint32_t, uint32_t, vector<uint8_t> > do_terrain(
        char name_prefix,
        std::string const& basename, std::string const& terrain, TerrainClasses const& terrain_classes,
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

        // Find the maximum cost in the dstar output, filtering out actual
        // obstacles
        float max_val = 0;
        for (size_t y = 0; y < ySize; ++y)
        {
            for (size_t x = 0; x < xSize; ++x)
            {
                float val = graph.getValue(x, y);
                if (val < 10000 && max_val < val)
                    max_val = val;
            }
        }

        // Now display
        vector<RGBColor> color_image;
        for (size_t i = 0; i < image.size(); ++i)
            color_image.push_back(RGBColor(image[i]));

        for (size_t y = 0; y < ySize; ++y)
        {
            for (size_t x = 0; x < xSize; ++x)
            {
                float val = graph.getValue(x, y);
                RGBColor color;
                if (val < 100000)
                    color = RGBColor(128, 128, 255 * (val / max_val));
                else
                    color = RGBColor(0, 0, 0);
                color_image[y * xSize + x] = color;
            }
        }

        string out = basename + "-dstar.tif";
        std::cerr << "  saving result in " << out << std::endl;
        saveColorImage(out, xSize, ySize, color_image);
    }
    
    list<VoronoiPoint> result;
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
        markPoints(border.second, xSize, color_image, RGBColor(100, 100, 200));

        vector<PointID> points;
        points.resize(result.size());
        transform(result.begin(), result.end(), points.begin(),
                boost::bind(&VoronoiPoint::center, _1));
        markPoints(points, xSize, color_image, RGBColor(128, 255, 128));

        string out = basename + "-border.tif";
        std::cerr << "  saving result in " << out << std::endl;
        saveColorImage(out, xSize, ySize, color_image);
    }

    CorridorExtractionState build_plan_state(PointID(x0, y0), PointID(x1, y1), algo.graph());
    { Profile profiler("extracting corridors and computing connections");
        skel.extractBranches(result, build_plan_state);
        skel.computeConnections(build_plan_state);
    }
    outputExtractionState(xSize, ySize, basename + "-branches.tif", image, build_plan_state);

    { Profile profiler("removing dead ends");
        build_plan_state.plan.createEndpointCorridor(build_plan_state.plan.getStartPoint(), false);
        build_plan_state.plan.createEndpointCorridor(build_plan_state.plan.getEndPoint(), true);
        build_plan_state.plan.removeDeadEnds();
    }

    Plan plan;
    { Profile profiler("doing the whole plan building at once");
        SkeletonExtraction skel(xSize, ySize);
        list<VoronoiPoint> skeleton = skel.processDStar(algo, x0, y0, expand);
        plan = skel.buildPlan(PointID(x0, y0), PointID(x1, y1), algo.graph(), skeleton);
    }

    for (size_t i = 0; i < plan.corridors.size(); ++i)
        plan.corridors[i].name = name_prefix + plan.corridors[i].name;

    cerr << plan.corridors.size() << " corridors found" << endl;

    outputPlan(xSize, ySize, basename, image, plan);

    plan.simplify();
    outputPlan(xSize, ySize, basename + "-simplified", image, plan);
    
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
    tie(original, xSize, ySize, image) = do_terrain('a', out_basename, terrain_file + ".tif", classes, x0, y0, x1, y1, expand);

    return 0;
    Plan blocked = do_terrain('b', out_basename + "-blocked", terrain_file + "-blocked.tif", classes, 446, 328, x1, y1, expand).get<0>();

    for (size_t i = 0; i < original.corridors.size(); ++i)
        original.corridors[i].name = string("a") + boost::lexical_cast<std::string>(i);
    for (size_t i = 0; i < blocked.corridors.size(); ++i)
        blocked.corridors[i].name = string("b") + boost::lexical_cast<std::string>(i);

    PlanMerge merger;

    outputPlan(xSize, ySize, out_basename + "-left", image, original);
    outputPlan(xSize, ySize, out_basename + "-right", image, blocked);

    merger.merge(original, blocked, 0.5, 0.2);

    cerr << merger.corridors.size() << " corridors in merged plan" << endl;
    outputPlan(xSize, ySize, out_basename + "-merge", image, merger);
    return 0;
}

