#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/dstar.hh"
#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>

using namespace Nav;
using namespace std;

static void outputGrownBorder(ostream& out, DStar const& algo, int x0, int y0, float expand)
{
    PointSet border = algo.solutionBorder(x0, y0, expand);

    out << std::endl;
    out << x0 << " " << y0 << " " << algo.getGoalX() << " " << algo.getGoalY() << " " << expand << std::endl;
    for (PointSet::iterator it = border.begin(); it != border.end(); ++it)
        out << it->x << " " << it->y << "\n";
    out << std::flush;
}

// Simple case of a map with an obstacle in the middle. The algorithm must
// solve a "symmetric" problem where two paths exist, each to go around the
// said obstacle
BOOST_AUTO_TEST_CASE( test_linear_symmetric )
{
    static const int Size = 101;
    static const int x0 = 50,
                 y0 = 5,
                 x1 = 50,
                 y1 = 95;
    static const float expand = 0.10;

    TraversabilityMap map(Size, Size);
    map.fill(15);
    for (int x = 20; x < 81; ++x)
        for (int y = 40; y < 61; ++y)
            map.setValue(x, y, 8);

    DStar algo(map);

    algo.initialize(x1, y1, x0, y0);
    BOOST_REQUIRE(algo.checkSolutionConsistency());

    ofstream out("multipath_linear_symmetric.txt");
    algo.graph().save(out);
    outputGrownBorder(out, algo, x0, y0, expand);
}

// Non symmetric case with two paths
// * a short, difficult path
// * a longer, half difficult, half simple
//
// Another part of the algorithm kicks in there: the one which takes the "out
// paths" of the growed region and propagate them afterwards.
BOOST_AUTO_TEST_CASE( test_multipath_heterogeneous )
{
    static const int Size = 101;
    static const int x0 = 75,
                 y0 = 5,
                 x1 = 75,
                 y1 = 95;
    static const float expand = 0.02;

    TraversabilityMap map(Size, Size);
    map.fill(15);
    for (int x = Size / 2; x < Size; ++x)
        for (int y = 0; y < Size; ++y)
            map.setValue(x, y, 8);
    for (int x = 0; x < Size / 2; ++x)
        for (int y = 0; y < Size / 2; ++y)
            map.setValue(x, y, 7);
    for (int x = 0; x < Size / 2; ++x)
        for (int y = Size / 2; y < Size; ++y)
            map.setValue(x, y, 15);

    DStar algo(map);

    algo.initialize(x1, y1, x0, y0);
    BOOST_REQUIRE(algo.checkSolutionConsistency());

    ofstream out("multipath_heterogeneous.txt");
    algo.graph().save(out);
    outputGrownBorder(out, algo, x0, y0, expand);
}

static int useful_rand(int max)
{
    return max * (static_cast<float>(rand()) / RAND_MAX);
}

// "Forest": random set of fixed-size obstacles, random starting and end points
BOOST_AUTO_TEST_CASE( test_multipath_forest )
{
    static const int Size = 101;
    static const int ObstacleCount = 200;
    static const int ObstacleRadius = 2;
    static const float expand = 0.02;

    srand(time(0));
    TraversabilityMap map(Size, Size);
    map.fill(15);

    for (int i = 0; i < ObstacleCount; ++i)
    {
        int x = useful_rand(Size - 2 * ObstacleRadius) + ObstacleRadius;
        int y = useful_rand(Size - 2 * ObstacleRadius) + ObstacleRadius;
        int klass = useful_rand(TraversabilityMap::CLASSES_COUNT / 2);

        for (int dx = - ObstacleRadius; dx < ObstacleRadius + 1; ++dx)
            for (int dy = - ObstacleRadius; dy < ObstacleRadius + 1; ++dy)
                map.setValue(x + dx, y + dy, klass);
    }
    int x0 = useful_rand(Size);
    int y0 = useful_rand(Size);
    int x1 = useful_rand(Size);
    int y1 = useful_rand(Size);

    DStar algo(map);

    algo.initialize(x1, y1, x0, y0);
    BOOST_REQUIRE(algo.checkSolutionConsistency());

    ofstream out("multipath_forest.txt");
    algo.graph().save(out);
    outputGrownBorder(out, algo, x0, y0, expand);

}


