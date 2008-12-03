#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/skeleton.hh"
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Nav;

void checkSimpleCorridorResult(Skeleton const& result, size_t xmin, size_t xmax, size_t w)
{
    int check[w];
    memset(check, 0, sizeof(int) * w);

    for (Skeleton::const_iterator it = result.begin(); it != result.end(); ++it)
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
    for (int x = 5; x < 30; ++x)
    {
        int ymin, ymax;
        if (x < 10)
        {
            ymin = 5 + x;
            ymax = 35 - x;
        }
        else if (x < 20)
        {
            ymin = 15;
            ymax = 25;
        }
        else if (x < 30)
        {
            ymin = 15 - (x - 20);
            ymax = 25 + (x - 20);
        }


        border.insert( PointID(x, ymin) );
        border.insert( PointID(x, ymax) );
        for (int y = ymin + 1; y < ymax; ++y)
            inside.insert( PointID(x, y) );
    }

    SkeletonExtraction skel(w, h);
    Skeleton result = skel.processEdgeSet(border, inside);

    displaySkeleton(cerr, result, w, h);
    // for (Skeleton::const_iterator it = result.begin(); it != result.end(); ++it)
    // {
    //     cerr << it->first << " [ ";
    //     set<PointID> const& parents = it->second.parents;
    //     for (set<PointID>::const_iterator it = parents.begin(); it != parents.end(); ++it)
    //         cerr << *it << " ";
    //     cerr << " ]" << endl;
    // }
    checkSimpleCorridorResult(result, 5, 30, w);
}


BOOST_AUTO_TEST_CASE( test_crossroad )
{
    const int w = 20;
    const int h = 30;

    int mid_x = w / 2;
    int mid_y = h / 2;
    int size  = 3;

    vector<uint8_t> img(w * h, 0);
    for (size_t i = 0; i < 5; ++i)
    {
        img[ (mid_y + size) * w + (mid_x - size) - i] = 255;
        img[ (mid_y - size) * w + (mid_x - size) - i] = 255;
        img[ (mid_y - size - i) * w + (mid_x - size)] = 255;
        img[ (mid_y + size + i) * w + (mid_x - size)] = 255;
        img[ (mid_y - size - i) * w + (mid_x + size)] = 255;
        img[ (mid_y + size + i) * w + (mid_x + size)] = 255;
    }
    for (int i = -size; i < size; ++i)
        img[ (mid_y + i) * w + (mid_x + size)] = 255;

    SkeletonExtraction skel(w, h);
    // PointSet result = skel.processEdgeImage(&img[0]);
}

