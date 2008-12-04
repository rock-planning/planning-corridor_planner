#include "skeleton.hh"
#include <stdint.h>
#include <iostream>
#include <queue>
#include <iomanip>

using namespace std;
using namespace Nav;

SkeletonExtraction::SkeletonExtraction(size_t width, size_t height)
    : width(width)
    , height(height)
    , heightmap(width * height)
{
}

SkeletonExtraction::~SkeletonExtraction()
{
}

void SkeletonExtraction::initializeHeightMap(size_t maxDist, PointSet const& inside)
{
    /* All of the heightmap is set to +maxDist+, except the border
     * which is set to 0
     */
    fill(heightmap.begin(), heightmap.end(), 0);
    for (PointSet::const_iterator it = inside.begin(); it != inside.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
            heightmap[it->y * width + it->x] = maxDist;
    }
}

PointID SkeletonExtraction::pointFromPtr(uint8_t* ptr) const
{
    uint32_t offset = ptr - &heightmap[0];
    return PointID(offset % width, offset / width);
}

multimap<PointID, PointID> SkeletonExtraction::propagateHeightMap(PointSet const& border)
{
    const int32_t addVal[8] = { 1, 2, 2, 1, 2, 2, 1, 1 };
    const int32_t displacement[8] = {
        -width, -width - 1, -width + 1,
        width, width - 1, width + 1, -1, +1 };

    CandidateSet candidates;
    std::map<uint8_t*, uint8_t*> parents;
    for (PointSet::const_iterator it = border.begin(); it != border.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
        {
            uint8_t* c_ptr = &heightmap[it->y * width + it->x];
            *c_ptr = 0;
            candidates.push( c_ptr );
            parents.insert( make_pair(c_ptr, c_ptr) );
        }
    }

    displayHeightMap(cerr);

    multimap<PointID, PointID> skeleton;
    while (!candidates.empty())
    {
        uint8_t* c_ptr   = candidates.top();
        uint8_t  c_value = *c_ptr;
        cerr << "taking " << pointFromPtr(c_ptr) << "(" << (void*)c_ptr << ")=" << (int)c_value << endl;
        candidates.pop();

        bool propagated = false, median = false;
        for (size_t i = 0; i < 8; ++i)
        {
            uint8_t* neighbour        = c_ptr   + displacement[i];
            uint32_t propagated_value = c_value + addVal[i]; 
            cerr <<  "  " << pointFromPtr(neighbour) << "(" << (void*)neighbour << ")=" << (int)*neighbour << " ~ " << propagated_value << endl;
            if (*neighbour > propagated_value) // needs to be propagated further
            {
                propagated = true;

                *neighbour = propagated_value;
                candidates.push( neighbour );
                parents[neighbour] = parents[c_ptr];
            }
            else if (*neighbour == propagated_value)
                propagated = true;
        }

        {
            int right  = c_ptr[displacement[7]];
            int left   = c_ptr[displacement[6]];
            int top    = c_ptr[displacement[3]];
            int bottom = c_ptr[displacement[0]];
            int straight_div = (right + left - 2 * c_value) + (top + bottom - 2 * c_value);
            if (right && left && top && bottom && straight_div < 0)
            {
                std::cerr << "  negative straight div " << straight_div << std::endl;
                median = true;
            }
            else
            {
                int top_right    = c_ptr[displacement[5]];
                int top_left     = c_ptr[displacement[4]];
                int bottom_right = c_ptr[displacement[2]];
                int bottom_left  = c_ptr[displacement[1]];
                int diag_div = (top_right + bottom_left - 2 * c_value) + (bottom_right + top_left - 2 * c_value);
                if (top_right && top_left && bottom_right && bottom_left && diag_div < 0)
                {
                    std::cerr << "  negative diag div " << diag_div << std::endl;
                    median = true;
                }
            }
        }

        if (median || !propagated)
        {
            PointID candidate_p = pointFromPtr(c_ptr);
            cerr << "peak " << (int)*c_ptr << " " << candidate_p << " (" << (void*)c_ptr << ") from\n";

            // Search for parents
            for (size_t i = 0; i < 8; ++i)
            {
                uint8_t* neighbour        = c_ptr   + displacement[i];
                uint32_t propagated_value = c_value - addVal[i]; 
                if (*neighbour == propagated_value)
                {
                    PointID parent_p = pointFromPtr(parents[neighbour]);
                    cerr << "  " << parent_p << " (" << (void*)parents[neighbour] << "), parent of " << pointFromPtr(neighbour) << endl;
                    skeleton.insert( std::make_pair(candidate_p, parent_p) );
                }
            }
        }
    }

    displayHeightMap(cerr);
    return skeleton;
}

void SkeletonExtraction::displayHeightMap(std::ostream& io) const
{
    io << "   ";
    for (int x = 0; x < width; ++x)
        io << " " << std::setw(3) << x;
    for (int y = 0; y < height; ++y)
    {
        io << std::setw(3) << y;
        for (int x = 0; x < width; ++x)
        {
            io << " " << std::setw(3) << (int)heightmap[y * width + x];
        }
        io << endl;
    }
}

Skeleton SkeletonExtraction::processEdgeSet(PointSet const& border, PointSet const& inside)
{
    initializeHeightMap(128, inside);

    Skeleton skeleton;
    multimap<PointID, PointID> result = propagateHeightMap(border);
    for (multimap<PointID, PointID>::const_iterator it = result.begin(); it != result.end(); ++it)
    {
        Skeleton::iterator skel_it = skeleton.find( it->first );
        if (skel_it == skeleton.end())
            skeleton.insert( make_pair(it->first, SkeletonInfo(it->second)) );
        else
            skel_it->second.parents.insert(it->second);
    }
    return skeleton;
}

void Nav::displaySkeleton(ostream& io, Skeleton const& skel, int w, int h)
{
    io << "  ";
    for (int x = 0; x < w; ++x)
        io << " " << std::setw(2) << x;
    io << endl;
    for (int y = 0; y < h; ++y)
    {
        io << std::setw(2) << y;
        for (int x = 0; x < w; ++x)
        {
            if (skel.count(PointID(x, y)))
                io << " " << std::setw(2) << 1;
            else
                io << "  -";
        }
        io << endl;
    }
}

