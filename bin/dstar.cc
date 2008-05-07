#include <gdal.h>
#include <nav/dstar.hh>

#include <vector>
#include <iostream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
using namespace std;
using namespace Nav;

int main(int argc, char** argv)
{
    if (argc < 6)
    {
	std::cerr 
	    << "usage: dstar terrain_file X0 Y0 X1 Y1\n"
            << "  computes the result of D* on the given map, searching\n"
            << "  for a path between (x0, y0) and (x1, y1)\n"
            << "\n"
            << "  the result is sent on standard output, in the following format:\n"
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
            << std::endl;
	exit(1);

    }
    GDALAllRegister();

    char const* input_path = argv[1];
    int x0 = boost::lexical_cast<int>(argv[2]);
    int y0 = boost::lexical_cast<int>(argv[3]);
    int x1 = boost::lexical_cast<int>(argv[4]);
    int y1 = boost::lexical_cast<int>(argv[5]);
    
    // Load the file and run D*
    TraversabilityMap* map = TraversabilityMap::load(input_path);
    DStar algo(*map);
    GridGraph const& graph = algo.graph();

    std::cerr << "applying D* on a " << graph.xSize() << "x" << graph.ySize() << " map" << std::endl;
    algo.initialize(x1, y1, x0, y0);

    // Then display the result
    std::cout << graph.xSize() << " " << graph.ySize() << std::endl;
    std::cout << x0 << " " << y0 << " " << x1 << " " << y1 << std::endl;
    for (int y = 0; y < (int)graph.ySize(); ++y)
    {
        for (int x = 0; x < (int)graph.xSize(); ++x)
        {
            NeighbourConstIterator parent = graph.parentsBegin(x, y);
            if (parent.isEnd())
                std::cout << x << " " << y << " " << graph.getValue(x, y) << " " << 0 << " " << 0 << " " << 0 << "\n";
            else
                std::cout << x << " " << y << " " << graph.getValue(x, y) << " " << parent.x() << " " << parent.y() << " " << (int)graph.getParents(x, y) << "\n";
        }
    }
    std::cout << std::flush;

    return 0;
}

