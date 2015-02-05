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
#include "skeleton.hh"
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

using namespace std;
using base::Time;
using namespace boost;
using namespace corridor_planner;

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

void markCurve(base::geometry::Spline<3>& curve, int xSize, vector<RGBColor> & pixels, RGBColor color)
{
    double t       = curve.getStartParam();
    double end_t   = curve.getEndParam();

    for (; t <= end_t; t += (end_t - t) / 200)
    {
        Eigen::Vector3d p = curve.getPoint(t);
        int x = lrint(p.x()), y = lrint(p.y());
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
        c.updateCurves();
        if (true)
        {
            cerr << "displaying " << c.name << " using curves" << endl;
            vector<PointID> points[2];
            base::geometry::Spline<3>* curves[2] = { &c.boundary_curves[0], &c.boundary_curves[1] };
            for (int i = 0; i < 2; ++i)
            {
                double const start = curves[i]->getStartParam();
                double end   = curves[i]->getEndParam();
                double delta = (end - start) / 200;
                for (double t = start; t < end; t += delta)
                {
                    Eigen::Vector3d p = curves[i]->getPoint(t);
                    points[i].push_back(PointID(lrint(p.x()), lrint(p.y())));
                }
            }
            markPoints(points[0], xSize, color_image, colors[corridor_idx]);
            markPoints(points[1], xSize, color_image, colors[corridor_idx]);

            std::vector<RGBColor> distance_colors;
            for (int i = 0; i <= 10; ++i)
                distance_colors.push_back( RGBColor(200 - 20 * i, 20 * i, 100));

            {
                double const start = c.median_curve.getStartParam();
                double end   = c.median_curve.getEndParam();
                double delta = (end - start) / 200;
                for (double t = start; t < end; t += delta)
                {
                    Eigen::Vector3d p = c.median_curve.getPoint(t);

                    int x = lrint(p.x()), y  = lrint(p.y());
                    int idx = x + y * xSize;

                    float width = c.width_curve.getPoint(t)(0, 0);
                    if (width <= 10)
                        color_image[idx] = distance_colors[(int) width];
                    else if (width > 1000)
                        throw std::runtime_error("width curve unstable");
                    else
                        color_image[idx] = RGBColor(0, 200, 100);

                }
            }
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

        Corridor::Connections const& connections = corridor.connections;
        Corridor::Connections::const_iterator conn_it;

        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            int target_idx = conn_it->target_idx;

            int in_side  = conn_it->this_side;
            int out_side = conn_it->target_side;
            dot << "  c" << corridor_idx << "_" << in_side
                << " -> c" << target_idx << "_" << out_side << " [weight=3];\n";
        }

        ostringstream node_setup;
        node_setup << "[fixedsize=true, width=0.1, height=0.1, label=\"\", style=filled, shape=circle, "
            << "color=\"" << colorToDot(color) << "\"];\n";

        dot << "  c" << corridor_idx << "_0 " << node_setup.str();
        dot << "  c" << corridor_idx << "_1 " << node_setup.str();

        dot << "  c" << corridor_idx << "_0" << " -> c" << corridor_idx
            << "_1" << " [label=\"" << corridor_name << "\", color=\""
            << colorToDot(color) << "\"]\n";
    }
    dot << "}\n";
}

void exportSkeletonHeightmap(std::vector<int16_t> const& heightmap,
        size_t xSize, size_t ySize, std::string const& out_file)
{
    int max_val = 0;

    vector<RGBColor> color_image;
    for (size_t y = 0; y < ySize; ++y)
    {
        for (size_t x = 0; x < xSize; ++x)
        {
            int value = heightmap[y * xSize + x];
            max_val = std::max(max_val, value);
        }
    }

    std::vector<RGBColor> image;
    for (size_t y = 0; y < ySize; ++y)
    {
        for (size_t x = 0; x < xSize; ++x)
        {
            int value = heightmap[y * xSize + x];
            if (value == 0)
                image.push_back(RGBColor(0, 0, 0));
            else
            {
                float normalized = static_cast<float>(value) / max_val;;
                image.push_back(RGBColor(128, 128, 255 * normalized));
            }

        }
    }

    std::cerr << "  saving result in " << out_file << std::endl;
    saveColorImage(out_file, xSize, ySize, image);
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

void exportNavigationFunction(nav_graph_search::GridGraph const& graph, size_t xSize, size_t ySize, std::vector<uint8_t> const& base_image, std::string const& out_file)
{
    // Find the maximum cost in the dstar output, filtering out actual
    // obstacles
    float min_val = 10000;
    float max_val = 0;
    for (size_t y = 0; y < ySize; ++y)
    {
        for (size_t x = 0; x < xSize; ++x)
        {
            float val = graph.getValue(x, y);
            if (val > 0 && val < 10000)
            {
                max_val = std::max(max_val, val);
                min_val = std::min(min_val, val);
            }
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
            if (val >= 100000)
                color = RGBColor(255, 255, 255);
            else if (val == 0)
                color = RGBColor(0, 0, 0);
            else
                color = RGBColor(128, 128, 255 * ((val - min_val) / (max_val - min_val)));
            color_image[y * xSize + x] = color;
        }
    }

    std::cerr << "  saving result in " << out_file << std::endl;
    saveColorImage(out_file, xSize, ySize, color_image);
}

void do_plan(char name_prefix,
        std::string const& basename, std::string const& terrain_file, std::string const& terrain_classes,
        float x0, float y0, float x1, float y1, float expand, float min_width)
{
    CorridorPlanner planner;
    planner.init(terrain_classes, terrain_file, min_width);
    planner.setWorldPositions(Eigen::Vector2d(x0, y0), Eigen::Vector2d(x1, y1));
    planner.setMarginFactor(expand);

    { Profile profiler("dstar");
        planner.computeDStar();
    }
    std::cerr << "  done ... checking solution consistency" << std::endl;
    planner.dstar_to_start->checkSolutionConsistency();
    planner.dstar_to_goal->checkSolutionConsistency();

    // Load the original terrain file, we will superimpose some data later on
    // it.
    GridGraph const& graph = planner.dstar_to_goal->graph();
    uint32_t const xSize = graph.xSize(), ySize = graph.ySize();
    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(terrain_file.c_str(), GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);

    vector<uint8_t> image(xSize * ySize);

    nav_graph_search::GridGraph& cost_to_goal  = planner.dstar_to_goal->graph();
    nav_graph_search::GridGraph& cost_to_start = planner.dstar_to_start->graph();
    {
        band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);
        string dstar_out = basename + "-dstar_to_goal.tif";
        exportNavigationFunction(cost_to_goal, xSize, ySize, image, dstar_out);
    }

    {
        band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);
        string dstar_out = basename + "-dstar_to_start.tif";
        exportNavigationFunction(cost_to_start, xSize, ySize, image, dstar_out);
    }

    {
        band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);
        string dstar_out = basename + "-dstar.tif";
        GridGraph common_graph(xSize, ySize);
        for (unsigned int y = 0; y < ySize; ++y)
            for (unsigned int x = 0; x < xSize; ++x)
                common_graph.setValue(x, y, cost_to_goal.getValue(x, y) + cost_to_start.getValue(x, y));
        exportNavigationFunction(common_graph, xSize, ySize, image, dstar_out);
    }


    { Profile profiler("skeleton");
        planner.extractSkeleton();
    }
    exportSkeletonHeightmap(planner.skeleton->heightmap, xSize, ySize, basename + "-skeleton-heightmap.tif");
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
    if (argc != 10)
    {
	std::cerr 
	    << "usage: plan_corridors <map_file> <cost_classes> <expand_factor> <x0> <y0> <x1> <y1> <min_width> <basename>\n"
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
    float expand = boost::lexical_cast<float>(argv[3]);
    float x0 = boost::lexical_cast<float>(argv[4]);
    float y0 = boost::lexical_cast<float>(argv[5]);
    float x1 = boost::lexical_cast<float>(argv[6]);
    float y1 = boost::lexical_cast<float>(argv[7]);
    float min_width = boost::lexical_cast<float>(argv[8]);
    std::string out_basename = argv[9];

    do_plan('c', out_basename, terrain_file, terrain_classes,
          x0, y0, x1, y1, expand, min_width);
 
    return 0;
}

