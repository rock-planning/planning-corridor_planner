#include <boost/test/auto_unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test/testsuite.hh"
#include "nav/skeleton.hh"
#include <algorithm>
#include <iostream>

using namespace std;

BOOST_AUTO_TEST_CASE( test_simple_corridor )
{
    const int w = 20;
    const int h = 30;

    vector<uint8_t> img(w * h, 0);
    memset(&img[w * 10], 255, w);
    memset(&img[w * 20], 255, w);

    SkeletonExtraction skel(w, h);
    vector<int> result = skel.extract(&img[0]);

    bool check[w];
    memset(check, 0, sizeof(bool) * w);
    for (vector<int>::iterator it = result.begin(); it != result.end(); ++it)
    {
        int x = *it % w;
        int y = *it / w;
        BOOST_REQUIRE_EQUAL(y, 15);
        check[x] = true;
    }

    for (int i = 2; i < w - 2; ++i)
        BOOST_REQUIRE(check[i]);
    BOOST_REQUIRE(!check[0]);
    BOOST_REQUIRE(!check[1]);
    BOOST_REQUIRE(!check[w - 2]);
    BOOST_REQUIRE(!check[w - 1]);
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
    vector<int> result = skel.extract(&img[0]);
}


