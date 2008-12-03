#include <gdal.h>
#include <nav/dstar.hh>

#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <fstream>
using namespace std;
using namespace Nav;

int main(int argc, char** argv)
{
    if (argc != 5 && argc != 8)
    {
	std::cerr 
	    << "usage: dstar terrain_file x1 y1 basename [x0 y0 expand_factor]\n"
            << "  computes the result of D* on the given map, with [x1, y1] as\n"
            << "  the goal. If x0, y0 and expand_factor are given, then we also\n"
            << "  compute the expanded zone of navigation around the optimal path\n"
            << "\n"
            << "  the result is saved into two blocks in <basename>.txt, separated\n"
            << "  by an empty line. The first one represents the D* output in the\n"
            << "  following format:\n"
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
            << "  the second one represents the outer part of the border of the\n"
            << "  filtered output\n"
            << std::endl;
	exit(1);

    }
    GDALAllRegister();

    char const* input_path = argv[1];
    int x1 = boost::lexical_cast<int>(argv[2]);
    int y1 = boost::lexical_cast<int>(argv[3]);
    std::string out_basename = argv[4];

    int x0 = boost::lexical_cast<int>(argv[5]);
    int y0 = boost::lexical_cast<int>(argv[6]);
    float expand = boost::lexical_cast<float>(argv[7]);
    
    // Load the file and run D*
    TraversabilityMap* map = TraversabilityMap::load(input_path);
    DStar algo(*map);
    GridGraph const& graph = algo.graph();

    std::cerr << "applying D* on a " << graph.xSize() << "x" << graph.ySize() << " map" << std::endl;
    algo.initialize(x1, y1, x0, y0);
    std::cerr << "done ... checking its consistency" << std::endl;
    algo.checkSolutionConsistency();

    // Then display the result
    std::cerr << "saving result in " << out_basename << ".txt" << std::endl;
    ofstream dstar_output((out_basename + ".txt").c_str());
    dstar_output << x0 << " " << y0 << " " << x1 << " " << y1 << std::endl;
    graph.save(dstar_output);
    dstar_output << std::endl;

    if (argc == 8)
    {
        std::cerr << "computing filtered border" << std::endl;
        PointSet border = algo.solutionBorder(x0, y0, expand).first;

        std::cerr << "saving filtered border" << std::endl;
        dstar_output << x0 << " " << y0 << " " << x1 << " " << y1 << " " << expand << std::endl;
        for (PointSet::iterator it = border.begin(); it != border.end(); ++it)
            dstar_output << it->x << " " << it->y << "\n";
        dstar_output << std::flush;
    }
    return 0;
}

