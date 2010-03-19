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

#include "corridor_planner.hh"
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

void markCurve(base::geometry::NURBSCurve3D& curve, int xSize, vector<RGBColor> & pixels, RGBColor color)
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
        if (c.updateCurves())
        {
            cerr << "displaying " << c.name << " using curves" << endl;
            vector<PointID> points;
            base::geometry::NURBSCurve3D* curves[3] = { &c.boundary_curves[0], &c.boundary_curves[1], &c.median_curve };
            for (int i = 0; i < 3; ++i)
            {
                double delta = curves[i]->getUnitParameter() / 10;
                double end   = curves[i]->getEndParam();
                for (double t = 0; t < end; t += delta)
                {
                    Eigen::Vector3d p = curves[i]->getPoint(t);
                    points.push_back(PointID(p.x(), p.y()));
                }
            }
            markPoints(points, xSize, color_image, colors[corridor_idx]);
        }
        else
        {
            cerr << "displaying " << c.name << " using raw points" << endl;
            markPoints(c.boundaries[0], xSize, color_image, colors[corridor_idx]);
            markPoints(c.boundaries[1], xSize, color_image, colors[corridor_idx]);

            vector<PointID> points;
            transform(c.voronoi.begin(), c.voronoi.end(),
                    back_inserter(points), bind(&VoronoiPoint::center, _1));
            markPoints(points, xSize, color_image, colors[corridor_idx]);
        }
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

        bool backwards = false;
        Corridor::Connections const& connections = corridor.connections;
        Corridor::Connections::const_iterator conn_it;

        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            int target_idx = conn_it->target_idx;

            int in_side  = conn_it->this_side;
            int out_side = conn_it->target_side;
            if (in_side == false)
                backwards = true;

            dot << "  c" << corridor_idx << "_" << in_side
                << " -> c" << target_idx << "_" << out_side << " [weight=3];\n";
        }

        ostringstream node_setup;
        node_setup << "[fixedsize=true, width=0.1, height=0.1, label=\"\", style=filled, shape=circle, "
            << "color=\"" << colorToDot(color) << "\"];\n";

        dot << "  c" << corridor_idx << "_0 " << node_setup.str();
        dot << "  c" << corridor_idx << "_1 " << node_setup.str();

        int in_side = backwards, out_side = !backwards;
        dot << "  c" << corridor_idx << "_" << in_side << " -> c" << corridor_idx
            << "_" << out_side << " [label=\"" << corridor_name << "\", color=\""
            << colorToDot(color) << "\"]\n";

        if (corridor.bidirectional)
        {
            dot << "  c" << corridor_idx << "_1 -> c" << corridor_idx << "_0 [color=\""
                << colorToDot(color) << "\"]\n";
        }
    }
    dot << "}\n";
}

void exportSkeleton(CorridorPlanner const& planner, size_t xSize, size_t ySize, std::vector<uint8_t> const& base_image, std::string const& out_file)
{
    vector<RGBColor> color_image;
    for (size_t i = 0; i < base_image.size(); ++i)
        color_image.push_back(RGBColor(base_image[i]));

    std::list<VoronoiPoint> const& points = planner.voronoi_points;
    for (std::list<VoronoiPoint>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        for (VoronoiPoint::BorderList::const_iterator border_it = it->borders.begin();
                border_it != it->borders.end(); ++border_it)
        {
            markPoints(*border_it, xSize, color_image, RGBColor(200, 200, 200));
        }
        color_image[it->center.y * xSize + it->center.x] = RGBColor(255, 255, 255);
    }

    std::cerr << "  saving result in " << out_file << std::endl;
    saveColorImage(out_file, xSize, ySize, color_image);
}

void exportNavigationFunction(CorridorPlanner const& planner, size_t xSize, size_t ySize, std::vector<uint8_t> const& base_image, std::string const& out_file)
{
    GridGraph const& graph = planner.dstar->graph();

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
    for (size_t i = 0; i < base_image.size(); ++i)
        color_image.push_back(RGBColor(base_image[i]));

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

    std::cerr << "  saving result in " << out_file << std::endl;
    saveColorImage(out_file, xSize, ySize, color_image);
}

void do_plan(char name_prefix,
        std::string const& basename, std::string const& terrain_file, std::string const& terrain_classes,
        int x0, int y0, int x1, int y1, float expand)
{
    CorridorPlanner planner;
    planner.init(terrain_classes, terrain_file);
    planner.setRasterPositions(Eigen::Vector2i(x0, y0), Eigen::Vector2i(x1, y1));
    planner.setMarginFactor(expand);

    { Profile profiler("dstar");
        planner.computeDStar();
    }
    std::cerr << "  done ... checking solution consistency" << std::endl;
    planner.dstar->checkSolutionConsistency();

    // Load the original terrain file, we will superimpose some data later on
    // it.
    GridGraph const& graph = planner.dstar->graph();
    uint32_t const xSize = graph.xSize(), ySize = graph.ySize();
    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(terrain_file.c_str(), GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);
    vector<uint8_t> image(xSize * ySize);
    band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);

    string dstar_out = basename + "-dstar.tif";
    exportNavigationFunction(planner, xSize, ySize, image, dstar_out);

    { Profile profiler("skeleton");
        planner.extractSkeleton();
    }
    exportSkeleton(planner, xSize, ySize, image, basename + "-skeleton.tif");

    { Profile profiler("plan building");
        planner.computePlan();
        cerr << planner.plan.corridors.size() << " corridors found" << endl;
    }
    outputPlan(xSize, ySize, basename + "-plan", image, planner.plan);

    { Profile profiler("plan simplification");
        planner.simplifyPlan();
        cerr << planner.plan.corridors.size() << " corridors found" << endl;
    }
    outputPlan(xSize, ySize, basename + "-plan-simplified", image, planner.plan);
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

    string terrain_file    = argv[1];
    string terrain_classes = argv[2];
    int x1 = boost::lexical_cast<int>(argv[3]);
    int y1 = boost::lexical_cast<int>(argv[4]);
    std::string out_basename = argv[5];
    int x0 = boost::lexical_cast<int>(argv[6]);
    int y0 = boost::lexical_cast<int>(argv[7]);
    float expand = boost::lexical_cast<float>(argv[8]);

    do_plan('c', out_basename, terrain_file, terrain_classes,
          x0, y0, x1, y1, expand);
 
    return 0;
}

