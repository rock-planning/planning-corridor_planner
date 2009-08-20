#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/skeleton.hh"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <string.h>

using namespace std;
using namespace Nav;

void checkSimpleCorridorResult(MedianLine const& result, size_t xmin, size_t xmax, size_t w)
{
    int check[w];
    memset(check, 0, sizeof(int) * w);

    for (MedianLine::const_iterator it = result.begin(); it != result.end(); ++it)
    {
        PointID p = it->first;
        BOOST_REQUIRE_EQUAL(p.y, 20);
        check[p.x] += 1;
    }

    for (size_t i = xmin; i < xmax - 1; ++i)
        BOOST_REQUIRE_MESSAGE(1 == check[i], "no median point for X == " << i);

    for (size_t i = 0; i < xmin; ++i)
        BOOST_REQUIRE_EQUAL(0, check[i]);
    for (size_t i = xmax; i < w; ++i)
        BOOST_REQUIRE_EQUAL(0, check[i]);
}

BOOST_AUTO_TEST_CASE( test_simple_corridor_set )
{
    const int w = 40;
    const int h = 40;

    PointSet border, inside;
    for (int x = 5; x < 35; ++x)
    {
        int ymin, ymax;
        if ((x >= 5 && x < 10) || (x >= 20 && x < 25))
        {
            int base_x = x / 5 * 5;
            ymin = 10 + (x - base_x);
            ymax = 30 - (x - base_x);
        }
        else if ((x >= 10 && x < 15) || (x >= 25 && x < 30))
        {
            ymin = 15;
            ymax = 25;
        }
        else if ((x >= 15 && x < 20) || (x >= 30 && x < 35))
        {
            int base_x = x / 5 * 5;
            ymin = 15 - (x - base_x);
            ymax = 25 + (x - base_x);
        }

        border.insert( PointID(x, ymin) );
        border.insert( PointID(x, ymax) );
        for (int y = ymin + 1; y < ymax; ++y)
            inside.insert( PointID(x, y) );
    }

    SkeletonExtraction skel(w, h);
    MedianLine result = skel.processPointSets(border, inside);
    displayMedianLine(cerr, result, 0, w, 0, h);

    cerr << "Corridor set:" << endl;
    Plan corridors( PointID( 5, 20 ),  PointID( 34, 20 ) );
    skel.buildPlan(corridors, result);
    cerr << corridors << endl;

    // One corridor and the corridor 0 which represents the outside
    BOOST_CHECK_EQUAL(2, corridors.corridors.size());
}


BOOST_AUTO_TEST_CASE( test_crossroad )
{
    const int w = 40;
    const int h = 40;

    PointSet border, inside;
    for (int x = 5; x < 30; ++x)
    {
        int ymin0 = 0, ymax0 = 0, ymin1 = 0, ymax1 = 0;
        if ((x >= 5 && x < 10) || (x >= 25 && x < 30)) // start and end: straight tunnel
        {
            ymin0 = 15;
            ymax0 = 25;
        }
        else if ((x >= 10 && x < 15)) // growing section
        {
            int base_x = x / 5 * 5;
            ymin0 = 15 - 2 * (x - base_x);
            ymax0 = 25 + 2 * (x - base_x);
            border.insert( PointID(x, ymin0 - 1) );
            border.insert( PointID(x, ymax0 + 1) );
        }
        else if ((x >= 15 && x < 20)) // split section
        {
            ymin0 = 5;
            ymax0 = 10;
            ymin1 = 30;
            ymax1 = 35;
        }
        else if (x >= 20 && x < 25) // merging section
        {
            int base_x = x / 5 * 5;
            ymin0 =  5 + 2 * (x - base_x);
            ymax0 = 35 - 2 * (x - base_x);
            border.insert( PointID(x + 1, ymin0 + 1) );
            border.insert( PointID(x + 1, ymax0 - 1) );
        }

        border.insert( PointID(x, ymin0) );
        border.insert( PointID(x, ymax0) );
        for (int y = ymin0 + 1; y < ymax0; ++y)
            inside.insert( PointID(x, y) );

        if (ymin1 != 0)
        {
            border.insert( PointID(x, ymin1) );
            border.insert( PointID(x, ymax1) );
            for (int y = ymin1 + 1; y < ymax1; ++y)
                inside.insert( PointID(x, y) );
        }
    }
    for (int y = 10; y <= 30; ++y)
    {
        border.insert( PointID(15, y) );
        border.insert( PointID(19, y) );
    }

    SkeletonExtraction skel(w, h);
    MedianLine result = skel.processPointSets(border, inside);
    displayMedianLine(cerr, result, 0, w, 0, h);

    cerr << "Corridor set:" << endl;
    Plan corridors(PointID( 5, 20 ),  PointID( 29, 20 ));
    skel.buildPlan(corridors, result);
    cerr << corridors << endl;

    // Four corridors and the corridor 0 which represents the outside
    BOOST_CHECK_EQUAL(5, corridors.corridors.size());
}

