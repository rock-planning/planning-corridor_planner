#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "dstar.hh"
#include "voronoi.hh"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <string.h>

using namespace std;
using namespace corridor_planner;

BOOST_AUTO_TEST_CASE( test_GridGraph_neighbour_iteration )
{
    GridGraph graph(10, 10);

    GridGraph::iterator it = graph.neighboursBegin(5, 5);

    PointID expected[] = {
        PointID(6, 5),
        PointID(6, 6), PointID(5, 6), PointID(4, 6),
        PointID(4, 5),
        PointID(4, 4), PointID(5, 4), PointID(6, 4)
    };

    for (int i = 0; !it.isEnd(); ++i, ++it)
        BOOST_REQUIRE_EQUAL(it.getTargetPoint(), expected[i]);
}

BOOST_AUTO_TEST_CASE( test_GridGraph_neighbour_iteration_with_mask )
{
    GridGraph graph(10, 10);

    GridGraph::iterator it = graph.neighboursBegin(5, 5, GridGraph::DIR_STRAIGHT);

    PointID expected[] = {
        PointID(6, 5), PointID(5, 6), PointID(4, 5), PointID(5, 4)
    };

    for (int i = 0; !it.isEnd(); ++i, ++it)
        BOOST_REQUIRE_EQUAL(it.getTargetPoint(), expected[i]);
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_direction )
{
    VoronoiPoint pt;

    Point<float> dir = pt.direction();
    BOOST_REQUIRE_EQUAL(0.0f, dir.x);
    BOOST_REQUIRE_EQUAL(0.0f, dir.y);

    PointID b0[3] = { PointID(-1, 1), PointID(-2, 1), PointID(0, 1) };
    pt.addBorder(b0, b0 + 3);

    BOOST_REQUIRE_THROW(pt.direction(), std::runtime_error);

    PointID b1[3] = { PointID(-1, -1), PointID(-2, -1), PointID(0, -1) };
    pt.addBorder(b1, b1 + 3);

    dir = pt.direction();
    BOOST_REQUIRE_CLOSE(0.0f, dir.x, 0.001f);
    BOOST_REQUIRE_CLOSE(1.0f, fabs(dir.y), 0.001f);
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_singleton )
{
    VoronoiPoint pt;
    BOOST_REQUIRE(pt.isSingleton());

    PointID b0[3] = { PointID(-1, 1), PointID(-2, 1), PointID(0, 1) };
    pt.addBorder(b0, b0 + 3);
    BOOST_REQUIRE(!pt.isSingleton());
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_offset )
{
    VoronoiPoint pt;
    PointID b0[3] = { PointID(-1, 1), PointID(-2, 1), PointID(0, 1) };
    pt.addBorder(b0, b0 + 3);
    pt.center = PointID(-1, -2);

    pt.offset(PointID(2, 1));
    BOOST_REQUIRE_EQUAL(1, pt.center.x);
    BOOST_REQUIRE_EQUAL(-1, pt.center.y);
    vector<PointID> b = pt.borders.front();
    BOOST_REQUIRE_EQUAL(1, b[0].x);
    BOOST_REQUIRE_EQUAL(0, b[1].x);
    BOOST_REQUIRE_EQUAL(2, b[2].x);
    BOOST_REQUIRE_EQUAL(2, b[0].y);
    BOOST_REQUIRE_EQUAL(2, b[1].y);
    BOOST_REQUIRE_EQUAL(2, b[2].y);
}

static bool border_equal(vector<PointID> const& b0, vector<PointID> const& b1)
{
    if (b0.size() != b1.size())
        return false;
    return std::equal(b0.begin(), b0.end(), b1.begin());
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_addBorderPoint )
{
    VoronoiPoint pt;

    pt.addBorderPoint( PointID(0, 1) );
    BOOST_REQUIRE_EQUAL(1, pt.borders.size());
    vector<PointID> b0;
    b0.push_back(PointID(0, 1));
    BOOST_REQUIRE_PREDICATE(border_equal, (b0)(pt.borders.front()));

    pt.addBorderPoint( PointID(0, -1) );
    BOOST_REQUIRE_EQUAL(2, pt.borders.size());
    BOOST_REQUIRE_PREDICATE(border_equal, (b0)(pt.borders.front()));
    vector<PointID> b1;
    b1.push_back(PointID(0, -1));
    BOOST_REQUIRE_PREDICATE(border_equal, (b1)(pt.borders.back()));

    pt.addBorderPoint( PointID(0, 0) );
    BOOST_REQUIRE_EQUAL(1, pt.borders.size());
    b0.push_back(PointID(0, 0));
    b0.push_back(PointID(0, -1));
    BOOST_REQUIRE_PREDICATE(border_equal, (b0)(pt.borders.front()));
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_isBorderAdjacent_PointID )
{
    VoronoiPoint pt;
    pt.addBorderPoint( PointID(0, 1) );
    pt.addBorderPoint( PointID(0, -1) );

    BOOST_REQUIRE( pt.isBorderAdjacent( PointID(0, 1)) );
    BOOST_REQUIRE( pt.isBorderAdjacent( PointID(0, -2)) );
    BOOST_REQUIRE( pt.isBorderAdjacent( PointID(1, 2)) );
    BOOST_REQUIRE( !pt.isBorderAdjacent( PointID(0, -3)) );
}

BOOST_AUTO_TEST_CASE( test_VoronoiPoint_isBorderAdjacent_VoronoiPoint )
{
    VoronoiPoint pt0;
    { PointID b0[3] = { PointID(-2, 1), PointID(-1, 1), PointID(0, 1) };
        pt0.addBorder(b0, b0 + 3); }
    { PointID b0[3] = { PointID(-2, -1), PointID(-1, -1), PointID(0, -1) };
        pt0.addBorder(b0, b0 + 3); }

    VoronoiPoint pt1;
    { PointID b0[3] = { PointID(1, 1), PointID(2, 1), PointID(3, 1) };
        pt1.addBorder(b0, b0 + 3); }
    BOOST_REQUIRE(!pt0.isBorderAdjacent(pt1));
    BOOST_REQUIRE(!pt1.isBorderAdjacent(pt0));
    { PointID b0[3] = { PointID(1, -1), PointID(2, -1), PointID(3, -1) };
        pt1.addBorder(b0, b0 + 3); }
    BOOST_REQUIRE(pt0.isBorderAdjacent(pt1));
    BOOST_REQUIRE(pt1.isBorderAdjacent(pt0));
}

