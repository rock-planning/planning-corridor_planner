#include <gdal.h>
#include <gdal_priv.h>
#include <nav/dstar.hh>
#include <nav/skeleton.hh>

#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <memory>
using namespace std;
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

void markPoints(PointSet const& points, int xSize, vector<RGBColor> & pixels, RGBColor color)
{
    for (PointSet::const_iterator point_it = points.begin(); point_it != points.end(); ++point_it)
    {
        int x = point_it->x, y = point_it->y;
        int idx = x + y * xSize;
        //cerr << x << " " << y << " " << (int)color.r << " " << (int)color.g << " " << (int)color.b << endl;

        pixels[idx] = color;
        if (x > 0)
            pixels[idx - 1] = color;
        if (y > 0)
            pixels[idx - xSize] = color;
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

    // Load the file and run D*
    TraversabilityMap* map = TraversabilityMap::load(terrain_file, classes);
    if (!map)
        return 1;

    DStar algo(*map, classes);
    GridGraph const& graph = algo.graph();
    uint32_t const xSize = graph.xSize(), ySize = graph.ySize();

    // Load the original terrain file, we will superimpose some data later on
    // it.
    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(terrain_file.c_str(), GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);
    vector<uint8_t> image(xSize * ySize);
    band->RasterIO(GF_Read, 0, 0, xSize, ySize, &image[0], xSize, ySize, GDT_Byte, 0, 0);

    std::cerr << "applying D* on a " << graph.xSize() << "x" << graph.ySize() << " map, target point is " << x1 << "x" << y1 << std::endl;
    algo.initialize(x1, y1);
    std::cerr << "  done ... checking solution consistency" << std::endl;
    algo.checkSolutionConsistency();

    {
        string dstar_out = out_basename + "-dstar.tif";
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
    
    if (argc == 9)
    {
        int x0 = boost::lexical_cast<int>(argv[6]);
        int y0 = boost::lexical_cast<int>(argv[7]);
        float expand = boost::lexical_cast<float>(argv[8]);

        std::cerr << "computing grown region" << std::endl;
        pair<PointSet, PointSet> border = algo.solutionBorder(x0, y0, expand);

        { vector<RGBColor> color_image;
            for (size_t i = 0; i < image.size(); ++i)
                color_image.push_back(RGBColor(image[i]));

            markPoints(border.first, xSize, color_image, RGBColor(128, 128, 255));
            string out = out_basename + "-border.tif";
            std::cerr << "  saving result in " << out << std::endl;
            saveColorImage(out, xSize, ySize, color_image);
        }

        std::cerr << "computing plan" << std::endl;
        SkeletonExtraction skel(xSize, ySize);
        MedianLine result = skel.processEdgeSet(border.first, border.second);
        Plan plan = skel.buildPlan(result);

        PointSet endpoints;
        endpoints.insert( PointID(x0, y0) );
        endpoints.insert( PointID(x1, y1) );
        plan.simplify(endpoints);

        cerr << plan.corridors.size() << " corridors found" << endl;

        { vector<RGBColor> color_image;
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
            }

            string corridor_out = out_basename + "-corridors.tif";
            cerr << "  saving image in " << corridor_out << endl;
            saveColorImage(corridor_out, xSize, ySize, color_image);
            string dot_out = out_basename + "-corridors.dot";
            cerr << "  saving result in " << dot_out << endl;
            ofstream dot(dot_out.c_str());
            dot << "graph {\n";
            for (size_t corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
            {
                RGBColor color = colors[corridor_idx];
                dot << "  c" << corridor_idx << "[color=\"#" << hex
                    << (int)color.r << (int)color.g << (int)color.b << "\"];\n" << dec;
                Corridor::Connections& connections = plan.corridors[corridor_idx].connections;
                Corridor::Connections::const_iterator conn_it;
                for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
                    dot << "  c" << corridor_idx << " -- c" << conn_it->get<1>() << ";\n";
            }
            dot << "}\n";

        }
    }
    return 0;
}

