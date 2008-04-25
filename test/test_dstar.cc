#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/dstar.hh"
#include <iostream>
#include <vector>
#include <cmath>

using namespace Nav;
using std::vector;
using std::make_pair;

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
    graph.setValue(100, 100, 0.214970);
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

    /** Check set(Source|Target)AsParent */
    graph.setParents(30, 30, GridGraph::LEFT);
    it = graph.neighboursBegin(30, 30);
    for (; it != graph.neighboursEnd(); ++it)
    {
        it.setSourceAsParent();
        NeighbourIterator parent_it = graph.parentsBegin(it.x(), it.y());
        BOOST_CHECK_EQUAL(30, parent_it.x());
        BOOST_CHECK_EQUAL(30, parent_it.y());
        BOOST_CHECK(++parent_it == graph.parentsEnd());
    }
    BOOST_REQUIRE_EQUAL(0, graph.getParents(30, 30));

    graph.setParents(31, 31, GridGraph::LEFT);
    graph.setParents(31, 30, GridGraph::LEFT);
    it = graph.neighboursBegin(30, 30);
    for (; it != graph.neighboursEnd(); ++it)
    {
        it.setTargetAsParent();
        NeighbourIterator parent_it = graph.parentsBegin(30, 30);
        BOOST_CHECK_EQUAL(it.x(), parent_it.x());
        BOOST_CHECK_EQUAL(it.y(), parent_it.y());
        BOOST_CHECK(++parent_it == graph.parentsEnd());
    }
    BOOST_REQUIRE_EQUAL(GridGraph::LEFT, graph.getParents(31, 31));
    BOOST_REQUIRE_EQUAL(0, graph.getParents(31, 30));
}

std::vector<float> dstarCosts()
{
    vector<float> basic_costs;
    for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
        basic_costs.push_back( DStar::costOfClass(i) );
    return basic_costs;
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
    vector<float> basic_costs = dstarCosts();

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
        (basic_costs[9] + basic_costs[10] + basic_costs[11] + basic_costs[15]) / 2 * sqrt(2),
        basic_costs[11] + basic_costs[15],
        (basic_costs[11] + basic_costs[12] + basic_costs[13] + basic_costs[15]) / 2 * sqrt(2),
        basic_costs[13] + basic_costs[15],
        (basic_costs[13] + basic_costs[14] + basic_costs[13] + basic_costs[15]) / 2 * sqrt(2),
        basic_costs[13] + basic_costs[15],
        (basic_costs[13] + basic_costs[12] + basic_costs[9] + basic_costs[15]) / 2 * sqrt(2)
    };

    NeighbourConstIterator it = graph.neighboursBegin(10, 10);
    for (int i = 0; it != graph.neighboursEnd(); ++it, ++i)
        BOOST_REQUIRE_CLOSE(expected_costs[i], algo.costOf(it), 0.01);
}

BOOST_AUTO_TEST_CASE( test_dstar_insert )
{
    TraversabilityMap map(100, 100);
    DStar algo(map);

    vector<float> basic_costs = dstarCosts();

    BOOST_REQUIRE_EQUAL(false, algo.updatedCostOf(10, 10, true).second);
    algo.insert(10, 10, 5);
    BOOST_REQUIRE(make_pair(5.0f, true) == algo.updatedCostOf(10, 10, true));
    algo.insert(10, 10, 4);
    BOOST_REQUIRE(make_pair(4.0f, true) == algo.updatedCostOf(10, 10, true));
    algo.insert(10, 10, 5);
    BOOST_REQUIRE(make_pair(4.0f, true) == algo.updatedCostOf(10, 10, true));
}

BOOST_AUTO_TEST_CASE( test_dstar_updated )
{
    TraversabilityMap map(100, 100);
    DStar algo(map);
    GridGraph& graph = algo.graph();

    vector<float> basic_costs = dstarCosts();

    graph.setValue(10, 10, 10);

    /** First, basic tests without parents */
    map.setValue(10, 10, 5);
    BOOST_REQUIRE_EQUAL(false, algo.updatedCostOf(10, 10, true).second);
    BOOST_REQUIRE_EQUAL(10, algo.updated(10, 10));
    BOOST_REQUIRE_EQUAL(10, graph.getValue(10, 10));
}

void checkDStarConsistency(DStar const& algo)
{
    GridGraph const& graph = algo.graph();

    for (int x = 0; x < (int)graph.xSize(); ++x)
        for (int y = 0; y < (int)graph.ySize(); ++y)
            if (x != algo.getGoalX() || y != algo.getGoalY())
            {
                NeighbourConstIterator parent = graph.parentsBegin(x, y);
                BOOST_CHECK(parent != graph.parentsEnd());
                BOOST_CHECK_CLOSE(graph.getValue(x, y), algo.costOf(parent) + parent.getValue(), 0.01);
            }
}

/** Checks the behaviour of D* on an empty map with constant traversability */
BOOST_AUTO_TEST_CASE( test_dstar_initialize_empty )
{
    TraversabilityMap map(11, 11);
    map.fill(10);
    DStar algo(map);
    GridGraph const& graph = algo.graph();


    /* Run on a problem whose result we know */
    algo.initialize(5, 5, 1, 1);
    checkDStarConsistency(algo);
    for (int i = 0; i < 5; ++i)
    {
        BOOST_REQUIRE_EQUAL(GridGraph::RIGHT, graph.getParents(i, 5));
        BOOST_REQUIRE_EQUAL(GridGraph::TOP_RIGHT, graph.getParents(i, i));
        BOOST_REQUIRE_EQUAL(GridGraph::TOP, graph.getParents(5, i));
        BOOST_REQUIRE_EQUAL(GridGraph::TOP_LEFT,  graph.getParents(10 - i, i));
        BOOST_REQUIRE_EQUAL(GridGraph::LEFT,  graph.getParents(10 - i, 5));
        BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM_LEFT,  graph.getParents(10 - i, 10 - i));
        BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM, graph.getParents(5, 10 - i));
        BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM_RIGHT, graph.getParents(i, 10 - i));
    }

    /* Then, try the update process. We first get a trajectory which goes
     * straight, and then add an obstacle in the middle, forcing the
     * trajectories to avoid it */
    algo.initialize(10, 5, 1, 5);
    checkDStarConsistency(algo);
    for (int i = 0; i < 10; ++i)
        BOOST_REQUIRE_EQUAL(GridGraph::RIGHT, graph.getParents(i, 5));
    for (int i = 0; i < 5; ++i)
    {
        BOOST_REQUIRE_EQUAL(GridGraph::TOP_RIGHT, graph.getParents(5 + i, i));
        BOOST_REQUIRE_EQUAL(GridGraph::TOP, graph.getParents(10, i));
        BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM, graph.getParents(10, 10 - i));
        BOOST_REQUIRE_EQUAL(GridGraph::BOTTOM_RIGHT, graph.getParents(5 + i, 10 - i));
    }
}

