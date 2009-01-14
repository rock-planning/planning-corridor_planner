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

void saveColorImage(string const& path, int xSize, int ySize, vector<uint8_t>& red, vector<uint8_t>& green, vector<uint8_t>& blue)
{
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Gtiff");
    auto_ptr<GDALDataset> output_set(driver->Create(path.c_str(), xSize, ySize, 3, GDT_Byte, 0));

    output_set->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &red[0], xSize, ySize, GDT_Byte, 0, 0);
    output_set->GetRasterBand(2)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &green[0], xSize, ySize, GDT_Byte, 0, 0);
    output_set->GetRasterBand(3)->RasterIO(GF_Write, 0, 0, xSize, ySize,
            &blue[0], xSize, ySize, GDT_Byte, 0, 0);
}

void markPolyline(PointSet const& points, int xSize, vector<uint8_t> & red, vector<uint8_t> & green, vector<uint8_t>& blue,
        uint8_t r, uint8_t g, uint8_t b)
{
    PointID last;
    for (PointSet::const_iterator point_it = points.begin(); point_it != points.end(); ++point_it)
    {
        PointID p = *point_it;
        if (point_it == points.begin())
        {
            red[p.x + p.y * xSize]   = r;
            green[p.x + p.y * xSize] = g;
            blue[p.x + p.y * xSize]  = b;
        }
        else
        {
            float dx = p.x - last.x;
            float dy = p.y - last.y;
            int steps = fabs(dx) > fabs(dy) ? ceil(fabs(dx)) : ceil(fabs(dy));

            dx /= steps;
            dy /= steps;

            for (int i = 0; i < steps; ++i)
            {
                int idx = last.x + i * dx + (last.y + i * dy) * xSize;

                red[idx]   = r;
                green[idx] = g;
                blue[idx]  = b;
            }
        }

        last = p;
    }
}

void markPoints(PointSet const& points, int xSize, vector<uint8_t> & red, vector<uint8_t> & green, vector<uint8_t>& blue,
        uint8_t r, uint8_t g, uint8_t b)
{
    for (PointSet::const_iterator point_it = points.begin(); point_it != points.end(); ++point_it)
    {
        red[point_it->x + point_it->y * xSize]   = r;
        green[point_it->x + point_it->y * xSize] = g;
        blue[point_it->x + point_it->y * xSize]  = b;
    }
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

        { vector<uint8_t> red(image.begin(), image.end()), green(image.begin(), image.end()), blue(image.begin(), image.end());
            markPoints(border.first, xSize, red, green, blue, 128, 128, 255);
            string out = out_basename + "-border.tif";
            std::cerr << "  saving result in " << out << std::endl;
            saveColorImage(out, xSize, ySize, red, green, blue);
        }

        std::cerr << "computing plan" << std::endl;
        SkeletonExtraction skel(xSize, ySize);
        MedianLine result = skel.processEdgeSet(border.first, border.second);
        Plan plan = skel.buildPlan(result);
        cerr << plan.corridors.size() << " corridors found" << endl;

        { vector<uint8_t> red(image.begin(), image.end()), green(image.begin(), image.end()), blue(image.begin(), image.end());
            Plan::corridor_iterator corridor_it;
            for (corridor_it = plan.corridors.begin(); corridor_it != plan.corridors.end(); ++corridor_it)
            {
                uint8_t r = 128;
                uint8_t g = 128;
                uint8_t b = 255;

                Corridor const& c = *corridor_it;
                MedianPoint::BorderList::const_iterator border_it;
                for (border_it = c.borders.begin(); border_it != c.borders.end(); ++border_it)
                    markPolyline(*border_it, xSize, red, green, blue, r, g, b);
            }

            string corridor_out = out_basename + "-corridors.tif";
            std::cerr << "  saving result in " << corridor_out << std::endl;
            saveColorImage(corridor_out, xSize, ySize, red, green, blue);
        }
    }
    return 0;
}

