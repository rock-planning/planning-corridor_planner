#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include <boost/tuple/tuple.hpp>
#include "nav/merge.hh"

using namespace corridor_planner;
using namespace std;

BOOST_AUTO_TEST_CASE( test_merge_non_covering )
{
    // Two simple parallel corridors
    Corridor top, bottom;
    for (int x = 0; x < 20; ++x)
    {
        MedianPoint top_median;
        top_median.addBorderPoint( PointID(x, 2) );
        top_median.addBorderPoint( PointID(x, 8) );
        top.add(PointID(x, 5), top_median);

        MedianPoint bottom_median;
        bottom_median.addBorderPoint( PointID(x, 12) );
        bottom_median.addBorderPoint( PointID(x, 18) );
        bottom.add(PointID(x, 15), bottom_median);
    }

    PlanMerge merge;
    merge.corridors.push_back(top);
    merge.corridors.push_back(bottom);
    BOOST_REQUIRE( !merge.mergeCorridors(0, 1, 0.5, 0.2) );
    BOOST_REQUIRE_EQUAL(2,  merge.corridors.size());
}

BOOST_AUTO_TEST_CASE( test_merge_not_enough_coverage )
{
    // Parallel corridors but not covering enough
    Corridor top, bottom;
    for (int x = 0; x < 20; ++x)
    {
        MedianPoint top_median;
        top_median.addBorderPoint( PointID(x, 2) );
        top_median.addBorderPoint( PointID(x, 8) );
        top_median.width = 3;
        top.add(PointID(x, 5), top_median);

        MedianPoint bottom_median;
        bottom_median.addBorderPoint( PointID(x, 5) );
        bottom_median.addBorderPoint( PointID(x, 11) );
        bottom_median.width = 3;
        bottom.add(PointID(x, 8), bottom_median);
    }

    PlanMerge merge;
    merge.corridors.push_back(top);
    merge.corridors.push_back(bottom);
    BOOST_REQUIRE( !merge.mergeCorridors(0, 1, 0.8, 0.2) );
    BOOST_REQUIRE_EQUAL(2,  merge.corridors.size());
}

BOOST_AUTO_TEST_CASE( test_merge_perpendicular_crossing )
{
    Corridor horizontal, vertical;
    for (int i = 0; i < 20; ++i)
    {
        MedianPoint h_median;
        h_median.addBorderPoint( PointID(i, 5) );
        h_median.addBorderPoint( PointID(i, 15) );
        horizontal.add(PointID(i, 10), h_median);

        MedianPoint v_median;
        v_median.addBorderPoint( PointID(5, i) );
        v_median.addBorderPoint( PointID(15, i) );
        vertical.add(PointID(10, i), v_median);
    }

    PlanMerge merge;
    merge.corridors.push_back(vertical);
    merge.corridors.push_back(horizontal);
    BOOST_REQUIRE( !merge.mergeCorridors(0, 1, 0.5, 0.1) );
    BOOST_REQUIRE_EQUAL(2,  merge.corridors.size());
}

/** In this test, we check that merging the same (geometric) corridor results in
 * one single corridor, with the connections at the endpoints moved to the
 * merged object
 */
BOOST_AUTO_TEST_CASE( test_merge_identical_corridors )
{
    Corridor a;
    for (int i = 0; i < 20; ++i)
    {
        MedianPoint median;
        median.addBorderPoint( PointID(i-3, i+3) );
        median.addBorderPoint( PointID(i+3, i-3) );
        a.add(PointID(i, i), median);
    }

    using boost::make_tuple;
    PlanMerge merge;
    merge.corridors.push_back(a);
    merge.corridors.push_back(a);

    { Corridor before, after;
        before.connections.push_back(make_tuple(PointID(-1, -1), 0, PointID(0, 0)));
        merge.corridors.push_back(before);
        after.connections.push_back(make_tuple(PointID(20, 20), 0, PointID(19, 19)));
        merge.corridors.push_back(after);

        merge.corridors[0].connections.push_back(make_tuple(PointID(0, 0), 2, PointID(-1, -1)));
        merge.corridors[0].connections.push_back(make_tuple(PointID(19, 19), 3, PointID(20, 20)));
    }

    { Corridor before, after;
        before.connections.push_back(make_tuple(PointID(-1, -1), 1, PointID(0, 0)));
        merge.corridors.push_back(before);
        after.connections.push_back(make_tuple(PointID(20, 20), 1, PointID(19, 19)));
        merge.corridors.push_back(after);

        merge.corridors[1].connections.push_back(make_tuple(PointID(0, 0), 4, PointID(-1, -1)));
        merge.corridors[1].connections.push_back(make_tuple(PointID(19, 19), 5, PointID(20, 20)));
    }

    // Do the merge and verify something happened
    BOOST_REQUIRE( merge.mergeCorridors(0, 1, 0.5, 0.1) );

    // Check that there is actually one and only one corridor more
    BOOST_REQUIRE_EQUAL(7,  merge.corridors.size());
    Corridor const& merged = merge.corridors[6];

    // And check it is identical to the defined corridor
    BOOST_REQUIRE(merged.borders == merge.corridors[0].borders);
    BOOST_REQUIRE(merged.median == merge.corridors[0].median);

    // Now check that the connections have been moved from the original
    // corridors to the merged one
    BOOST_REQUIRE_EQUAL(4, merged.connections.size());

    boost::tuple<PointID, int, PointID> expected[4] = {
        make_tuple(PointID(0, 0), 2, PointID(-1, -1)),
        make_tuple(PointID(0, 0), 4, PointID(-1, -1)),
        make_tuple(PointID(19, 19), 3, PointID(20, 20)),
        make_tuple(PointID(19, 19), 5, PointID(20, 20)) 
    };
    Corridor::Connections::const_iterator it = merged.connections.begin();

    for (int i = 0; i < 4; ++i, ++it)
    {
        boost::tuple<PointID, int, PointID> val = *it;
        BOOST_REQUIRE(expected[i].get<0>() == val.get<0>());
        BOOST_REQUIRE(expected[i].get<1>() == val.get<1>());
        BOOST_REQUIRE(expected[i].get<2>() == val.get<2>());

        int other_idx = val.get<1>();
        Corridor const& other = merge.corridors[other_idx];
        boost::tuple<PointID, int, PointID> other_val = *other.connections.begin();
        BOOST_REQUIRE_EQUAL(1, other.connections.size());
        BOOST_REQUIRE_EQUAL(val.get<2>(), other_val.get<0>());
        BOOST_REQUIRE_EQUAL(6, other_val.get<1>());
        BOOST_REQUIRE_EQUAL(val.get<0>(), other_val.get<2>());
    }
}

/** In this test, we check that two corridors that overal enough are merged */
BOOST_AUTO_TEST_CASE( test_merge_near_corridors )
{
    Corridor a, b;
    for (int i = 0; i < 20; ++i)
    {
        MedianPoint a_median;
        a_median.addBorderPoint( PointID(i-3, i+3) );
        a_median.addBorderPoint( PointID(i+3, i-3) );
        a_median.width = 3;
        a.add(PointID(i, i), a_median);

        MedianPoint b_median;
        b_median.addBorderPoint( PointID(i-2, i+2) );
        b_median.addBorderPoint( PointID(i+4, i-4) );
        b_median.width = 3;
        b.add(PointID(i+1, i-1), b_median);
    }

    PlanMerge merge;
    merge.corridors.push_back(a);
    merge.corridors.push_back(b);
    BOOST_REQUIRE( merge.mergeCorridors(0, 1, 0.5, 0.1) );
    BOOST_REQUIRE_EQUAL(3,  merge.corridors.size());
}

/** In this test case, we try to merge two different corridors which has only a
 * section in the middle in common
 */
BOOST_AUTO_TEST_CASE( test_fork_merge )
{
    Corridor a, b;
    for (int x = 0; x <= 8; ++x)
    {
        MedianPoint a_median;
        a_median.addBorderPoint( PointID(x, 10-x) );
        a_median.addBorderPoint( PointID(x, 6-x) );
        a_median.width = 2;
        a.add(PointID(x, 8-x), a_median);

        MedianPoint b_median;
        b_median.addBorderPoint( PointID(x, -10+x) );
        b_median.addBorderPoint( PointID(x, -6+x) );
        b_median.width = 2;
        b.add(PointID(x, -8+x), b_median);
    }
    for (int x = 9; x <= 15; ++x)
    {
        MedianPoint median;
        median.addBorderPoint( PointID(x, 2) );
        median.addBorderPoint( PointID(x, -2) );
        median.width = 2;
        a.add(PointID(x, 0), median);
        b.add(PointID(x, 0), median);
    }
    for (int x = 15; x <= 23; ++x)
    {
        MedianPoint a_median;
        a_median.addBorderPoint( PointID(x, 2+x-15) );
        a_median.addBorderPoint( PointID(x, -2+x-15) );
        a_median.width = 2;
        a.add(PointID(x, x-15), a_median);

        MedianPoint b_median;
        b_median.addBorderPoint( PointID(x, 2-x+15) );
        b_median.addBorderPoint( PointID(x, -2-x+15) );
        b_median.width = 2;
        b.add(PointID(x, -x+15), b_median);
    }

    PlanMerge merge;
    merge.corridors.push_back(a);
    merge.corridors.push_back(b);
    BOOST_REQUIRE( merge.mergeCorridors(0, 1, 0.5, 0.1) );
    BOOST_REQUIRE_EQUAL(7,  merge.corridors.size());

    for (int i = 2; i < 7; ++i)
        cerr << merge.corridors[i] << endl;

}


