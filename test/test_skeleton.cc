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

    vector<uint8_t> img(w * h);
    memset(&img[w * 10], 255, w);
    memset(&img[w * 20], 255, w);

    SkeletonExtraction skel(w, h);
    vector<int> result = skel.extract(&img[0]);

    for (vector<int>::iterator it = result.begin(); it != result.end(); ++it)
        cout << *it % w << " " << *it / w << endl;
}

