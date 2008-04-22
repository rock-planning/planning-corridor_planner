#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/dstar.hh"
#include <iostream>
#include <vector>
#include <cmath>

using namespace Nav;
using std::vector;

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

void check_neighbour_iteration(int x, int y, GridGraph& graph, const int* offsets, int count)
{
    NeighbourIterator it = graph.neighboursBegin(x, y);
    for (int i = 0; i < count; ++i)
    {
        BOOST_REQUIRE(graph.neighboursEnd() != it);
        BOOST_REQUIRE_EQUAL(x + offsets[i * 2], it.x());
        BOOST_REQUIRE_EQUAL(y + offsets[i * 2 + 1], it.y());
        ++it;
    }
    BOOST_REQUIRE(graph.neighboursEnd() == it);
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

    /** Check iteration on parents */
    NeighbourIterator it = graph.parentsBegin(10, 10);
    BOOST_REQUIRE_EQUAL(10, it.sourceX());
    BOOST_REQUIRE_EQUAL(10, it.sourceY());
    BOOST_REQUIRE(graph.parentsEnd() == it);

    it = graph.parentsBegin(101, 101);
    BOOST_REQUIRE(graph.parentsEnd() != it);
    BOOST_REQUIRE_EQUAL(GridGraph::TOP_RIGHT, it.getNeighbour());
    BOOST_REQUIRE_EQUAL(102, it.x());
    BOOST_REQUIRE_EQUAL(102, it.y());
    ++it;
    BOOST_REQUIRE(graph.parentsEnd() == it);

    /** Check iteration on neighbours */
    {
        const int expected_values[] = { 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, -1, 1, -1 };
        check_neighbour_iteration(10, 10, graph, expected_values, 8);
    }
    {
        const int expected_values[] = { 1, 0, 1, 1, 0, 1 };
        check_neighbour_iteration(0, 0, graph, expected_values, 3);
    }
    {
        const int expected_values[] = { 1, 0, 1, 1, 0, 1, 0, -1, 1, -1 };
        check_neighbour_iteration(0, 100, graph, expected_values, 5);
    }
    {
        const int expected_values[] = { 1, 0, 0, -1, 1, -1 };
        check_neighbour_iteration(0, 199, graph, expected_values, 3);
    }
    {
        const int expected_values[] = { 0, 1, -1, 1, -1, 0 };
        check_neighbour_iteration(199, 0, graph, expected_values, 3);
    }
    {
        const int expected_values[] = { 0, 1, -1, 1, -1, 0, -1, -1, 0, -1 };
        check_neighbour_iteration(199, 100, graph, expected_values, 5);
    }
    {
        const int expected_values[] = { -1, 0, -1, -1, 0, -1 };
        check_neighbour_iteration(199, 199, graph, expected_values, 3);
    }

    /** Check convertion from non-const iterators to const iterators */
    NeighbourConstIterator cv_it = graph.parentsBegin(101, 101);
    BOOST_REQUIRE(cv_it == graph.parentsBegin(101, 101));
}

BOOST_AUTO_TEST_CASE( test_dstar_cost )
{
    TraversabilityMap map(100, 100);
    DStar algo(map);

    float min_cost = 1.0f;
    float max_cost = std::pow(2, 1 / 0.01);

    BOOST_REQUIRE_CLOSE(min_cost, DStar::costOfClass(TraversabilityMap::CLASSES_COUNT - 1), 0.01);
    BOOST_REQUIRE_CLOSE(max_cost, DStar::costOfClass(0), 0.01);

    map.setValue(10, 10, 0);
    BOOST_REQUIRE_CLOSE(max_cost, algo.costOf(10, 10), 0.01);
    map.setValue(10, 10, TraversabilityMap::CLASSES_COUNT - 1);
    BOOST_REQUIRE_CLOSE(min_cost, algo.costOf(10, 10), 0.01);

    /** From now, we assume that GridGraph::costOfClass(x) behaves properly and
     * we extract the cost of each traversability classes
     */
    float basic_costs[TraversabilityMap::CLASSES_COUNT];
    for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
        basic_costs[i] = DStar::costOfClass(i);

    GridGraph const& graph = algo.graph();

    map.setValue(10, 10, 15);

    map.setValue(11, 10, 9);
    map.setValue(11, 11, 10);
    map.setValue(10, 11, 11);
    map.setValue(9,  11, 12);
    map.setValue(9,  10, 13);
    map.setValue(9,   9, 14);
    map.setValue(10,  9, 13);
    map.setValue(11,  9, 12);

    float expected_costs[] = 
    { 
        basic_costs[9] + basic_costs[15],
        (basic_costs[9] + basic_costs[10] + basic_costs[11] + basic_costs[15]) / 2 * sqrt(2) / 2,
        basic_costs[11] + basic_costs[15],
        (basic_costs[11] + basic_costs[12] + basic_costs[13] + basic_costs[15]) / 2 * sqrt(2) / 2,
        basic_costs[13] + basic_costs[15],
        (basic_costs[13] + basic_costs[14] + basic_costs[13] + basic_costs[15]) / 2 * sqrt(2) / 2,
        basic_costs[13] + basic_costs[15],
        (basic_costs[13] + basic_costs[12] + basic_costs[9] + basic_costs[15]) / 2 * sqrt(2) / 2
    };

    NeighbourConstIterator it = graph.neighboursBegin(10, 10);
    for (int i = 0; it != graph.neighboursEnd(); ++it, ++i)
        BOOST_REQUIRE_CLOSE(expected_costs[i], algo.costOf(it), 0.01);
}

