#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/dstar.hh"
#include <iostream>

using namespace Nav;

/* This test checks the access and change of values in traversability maps.  It
 * generates RANDOM_TEST_COUNT position at which the traversability value
 * should be changed using setValue, and verify the value returned by getValue
 */
BOOST_AUTO_TEST_CASE( test_traversability_map )
{
    static const int RANDOM_TEST_COUNT = 10000;
    TraversabilityMap map(200, 200);

    uint8_t expected_values[200][200];
    memset(expected_values, 0, 200*200);

    /* First, a simple test */
    for (int x = 99; x < 102; ++x)
        for (int y = 99; y < 102; ++y)
            BOOST_REQUIRE_EQUAL(0, map.getValue(x, y));
    map.setValue(100, 100, 0xF);
    for (int x = 99; x < 102; ++x)
        for (int y = 99; y < 102; ++y)
        {
            if (x != 100 || y != 100)
                BOOST_REQUIRE_EQUAL(0, map.getValue(x, y));
            else
                BOOST_REQUIRE_EQUAL(0xF, map.getValue(x, y));
        }

    expected_values[100][100] = 0xF;
    for (int i = 0; i < RANDOM_TEST_COUNT; ++i)
    {
        uint8_t value = rand() * TraversabilityMap::CLASSES_COUNT / RAND_MAX;
        BOOST_REQUIRE(value < TraversabilityMap::CLASSES_COUNT);
        size_t  x = rand() * 200 / RAND_MAX;
        size_t  y = rand() * 200 / RAND_MAX;

        map.setValue(x, y, value);
        expected_values[x][y] = value;
        BOOST_REQUIRE_EQUAL(value, map.getValue(x, y));
    }

    /* and now check global consistency */
    for (int x = 0; x < 200; ++x)
        for (int y = 0; y < 200; ++y)
            BOOST_REQUIRE_EQUAL(expected_values[x][y], map.getValue(x, y));
}

BOOST_AUTO_TEST_CASE( test_grid_graph )
{
    GridGraph graph(200, 200);

    /** First, simple checks for access and mutation */
    BOOST_REQUIRE_EQUAL(0, graph.getParents(100, 100));
    graph.getParents(100, 100) = 0xA;
    BOOST_REQUIRE_EQUAL(0xA, graph.getParents(100, 100));
    graph.setParent(101, 101, GridGraph::BOTTOM_LEFT);
    BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM_LEFT, graph.getParents(101, 101));
    graph.setParent(101, 101, GridGraph::TOP_RIGHT);
    BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM_LEFT|GridGraph::TOP_RIGHT, graph.getParents(101, 101));
    graph.clearParent(101, 101, GridGraph::BOTTOM_LEFT);
    BOOST_REQUIRE_EQUAL(GridGraph::TOP_RIGHT, graph.getParents(101, 101));
    BOOST_REQUIRE_EQUAL(0, graph.getValue(100, 100));
    graph.getValue(100, 100) = 0.214970;
    BOOST_CHECK_CLOSE(0.214970F, graph.getValue(100, 100), 0.0001);

    /** Now, check iteration */
    NeighbourIterator it = graph.parentsBegin(10, 10);
    BOOST_REQUIRE_EQUAL(10, it.nodeX());
    BOOST_REQUIRE_EQUAL(10, it.nodeY());
    BOOST_REQUIRE(graph.parentsEnd() == it);

    it = graph.parentsBegin(101, 101);
    BOOST_REQUIRE(graph.parentsEnd() != it);
    BOOST_REQUIRE_EQUAL(2, it.getNeighbour());
    BOOST_REQUIRE_EQUAL(102, it.x());
    BOOST_REQUIRE_EQUAL(102, it.y());
    ++it;
    BOOST_REQUIRE(graph.parentsEnd() == it);
}

