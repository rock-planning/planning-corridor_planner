#include "skeleton.hh"
#include <stdint.h>
#include <iostream>
#include <queue>
#include <iomanip>
#include <limits>
#include <list>

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

void SkeletonExtraction::initializeHeightMap(PointSet const& inside)
{
    /* All of the heightmap is initialized with 0, then the inside is set to
     * the maximum value allowed by height_t. Finally, the border will be set to
     * 1 by propagateHeightMap
     */
    height_t maxDist = 999; // std::numeric_limits<height_t>::max();

    fill(heightmap.begin(), heightmap.end(), 0);
    for (PointSet::const_iterator it = inside.begin(); it != inside.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
            heightmap[it->y * width + it->x] = maxDist;
    }
}

PointID SkeletonExtraction::pointFromPtr(height_t const* ptr) const
{
    uint32_t offset = ptr - &heightmap[0];
    return PointID(offset % width, offset / width);
}

MedianLine SkeletonExtraction::propagateHeightMap(PointSet const& border)
{
    const int32_t addVal[8] = { 1, 2, 2, 1, 2, 2, 1, 1 };
    const int32_t displacement[8] = {
        -width, -width - 1, -width + 1,
        width, width - 1, width + 1, -1, +1 };

    // During propagation, we need to maintain only three sets:
    //  * the set of lowest distance d (candidates)
    //  * the set of d + 1 (direct propagation) (borders[0])
    //  * the set of d + 2 (diagonal propagation) (borders[1])
    //
    // There is no other distance to be considered. We therefore do not need to
    // maintain a priority queue or whatever, just those three sets. Still, a
    // point can be promoted from d + 2 to d + 1, so we need to keep them as
    // std::set.
    typedef std::set<height_t*> CandidateSet;
    CandidateSet candidates;
    CandidateSet borders[2];

    // Initialize +candidates+ and +parents+ with the border points
    std::map<height_t*, set<PointID> > parents;
    for (PointSet::const_iterator it = border.begin(); it != border.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
        {
            height_t* c_ptr = &heightmap[it->y * width + it->x];
            *c_ptr = 1;
            candidates.insert( c_ptr );
            parents[c_ptr].insert( pointFromPtr(c_ptr) );
        }
    }

    displayHeightMap(cerr);

    // The points that are part of the median line. The parents are stored in
    // the +parents+ map.
    CandidateSet skeleton;

    // Outer loop. This will leave if all three +candidates+, +borders[0]+ and
    // +borders[1]+ are empty.
    while (!candidates.empty())
    {
        // Inner loop. Process all points in +candidates+
        while (!candidates.empty())
        {
            height_t* c_ptr   = *candidates.begin();
            height_t  c_value = *c_ptr;
            cerr << "taking " << pointFromPtr(c_ptr) << "(" << (void*)c_ptr << ")=" << (int)c_value << endl;
            cerr << "  with parents";

            PointSet const& parent_set = parents[c_ptr];
            for (PointSet::const_iterator it = parent_set.begin(); it != parent_set.end(); ++it)
                cerr << " " << *it;
            cerr << endl;

            candidates.erase(candidates.begin());

            // Look at the neighbours. Three situations:
            //  * the propagated cost (current_cost + edge_cost) is lower. The
            //    target node is nearer to the parent of the current node.
            //    Propagate.
            //  * the cost of the target node is the same than the propagated
            //    cost. The parents of the current node are at the same distance
            //    of the neighbour than its current parents. Merge.
            //  * the cost of the neighbour is the same than the current cost.
            //    The actual Voronoi point is in-between the current node and
            //    the neighbour. We don't have half-coordinates, so merge also.
            //    Post-processing will remove points that have the same parents.
            for (size_t i = 0; i < 8; ++i)
            {
                height_t* neighbour        = c_ptr   + displacement[i];
                height_t propagated_value = c_value + addVal[i]; 
                cerr <<  "  " << pointFromPtr(neighbour) << "(" << (void*)neighbour << ")=" << (int)*neighbour << " ~ " << propagated_value << endl;
                if (*neighbour > propagated_value) // needs to be propagated further
                {
                    *neighbour = propagated_value;
                    if (addVal[i] == 1)
                    {
                        borders[1].erase(neighbour);
                        borders[0].insert(neighbour);
                    }
                    else
                        borders[1].insert(neighbour);

                    parents[neighbour] = parents[c_ptr];
                }
                else if (*neighbour == propagated_value || (addVal[i] == 1 && *neighbour == c_value))
                {
                    cerr <<  "   ... found border ?" << flush;
                    bool is_neighbour = false;
                    for (PointSet::const_iterator parent_it = parents[neighbour].begin(); parent_it != parents[neighbour].end(); ++parent_it)
                    {
                        cerr << " " << *parent_it << flush;
                        if (parent_it->isNeighbour(parents[c_ptr]))
                        {
                            is_neighbour = true;
                            break;
                        }
                    }

                    if (!is_neighbour)
                    {
                        cerr << " yes" << endl;
                        parents[neighbour].insert( parents[c_ptr].begin(), parents[c_ptr].end() );
                        skeleton.insert( neighbour );
                    }
                    else
                    {
                        cerr << " no" << endl;
                    }
                }
            }
        }

        if (borders[0].empty())
        {
            candidates.swap(borders[1]);
            borders[0].clear();
        }
        else
        {
            candidates.swap(borders[0]);
            borders[0].swap(borders[1]);
        }
    }

    displayHeightMap(cerr);

    MedianLine result;
    for (CandidateSet::const_iterator it = skeleton.begin(); it != skeleton.end(); ++it)
    {
        PointID p = pointFromPtr(*it);
        MedianPoint& info = result[p];

        info.distance = **it;
        for (PointSet::const_iterator parent_it = parents[*it].begin(); parent_it != parents[*it].end(); ++parent_it)
            info.border_points.insert(*parent_it);
    }

    return result;
}

void SkeletonExtraction::displayHeightMap(std::ostream& io) const
{
    io << "   ";
    for (int x = 0; x < width; ++x)
        io << " " << std::setw(3) << x;
    io << endl;
    for (int y = 0; y < height; ++y)
    {
        io << std::setw(3) << y;
        for (int x = 0; x < width; ++x)
        {
            int val = (int)heightmap[y * width + x];
            if (val)
                io << " " << std::setw(3) << val;
            else
                io << "   -";
        }
        io << endl;
    }
}

MedianLine SkeletonExtraction::processEdgeSet(PointSet const& border, PointSet const& inside)
{
    initializeHeightMap(inside);
    MedianLine line = propagateHeightMap(border);

    for (MedianLine::const_iterator it = line.begin(); it != line.end(); ++it)
        Corridor corridor(it->first, it->second);

    return line;
}

//Skeleton SkeletonExtraction::extractCorridorSet(MedianLine const& points)
//{
//    // Extract the set of corridors from the provided points. We use the
//    // heightmap to mark which point is in which corridor (and therefore have a
//    // fast neighbouring search). The value in the height map is the corridor
//    // index
//    fill(heightmap.begin(), heightmap.end(), 0);
//
//    vector< list<PointID> > corridors;
//    corridors.push_back( list<PointID>() );
//
//    const int32_t displacement[8] = {
//        -width, -width - 1, -width + 1,
//        width, width - 1, width + 1, -1, +1 };
//    vector<int> adjacents;
//    adjacents.reserve(8);
//
//    for (MedianLine::const_iterator median_it = points.begin(); median_it != points.end(); ++median_it)
//    {
//        PointSet const& border_points = median_it->second.border_points;
//
//        for (PointSet::const_iterator it = border_points.begin(); it != border_points.end(); ++it)
//        {
//            height_t* c_ptr = it->y * width + it->x;
//
//            adjacents.clear();
//            for (int i = 0; i < 8; ++i)
//            {
//                if (c_ptr[displacement[i]])
//                    adjacents.push_back(displacement[i]);
//            }
//            if (adjacents.empty())
//            {
//            }
//        }
//    }
//}

void Nav::displayMedianLine(ostream& io, MedianLine const& skel, int w, int h)
{
    cerr << "Bitmap:" << endl;
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

    cerr << "Parent list:" << std::endl;
    for (MedianLine::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        cerr << "  " << it->first << " [ ";
        set<PointID> const& parents = it->second.border_points;
        for (set<PointID>::const_iterator it = parents.begin(); it != parents.end(); ++it)
            cerr << *it << " ";
        cerr << " ]" << endl;
    }
}

Corridor::Corridor(PointID const& p, MedianPoint const& info)
{
    median[p] = info;
    typedef list< list<PointID> > BorderList;
    BorderList borders;

    PointSet const& points = info.border_points;
    for (PointSet::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        vector<BorderList::iterator> adjacent_borders;
        for (BorderList::iterator border_it = borders.begin(); border_it != borders.end(); ++border_it)
        {
            if (p.isNeighbour(*border_it))
                adjacent_borders.push_back(border_it);
        }

        if (adjacent_borders.empty())
        {
            BorderList::value_type new_border;
            new_border.push_back(*it);
            borders.push_back(new_border);
        }
        else
        {
            list<PointID>& merged = *adjacent_borders[0];
            for (vector<BorderList::iterator>::iterator border_it = ++adjacent_borders.begin();
                    border_it != adjacent_borders.end(); ++border_it)
            {
                merged.splice(merged.end(), **border_it);
            }
            merged.push_back(p);
        }
    }

    if (borders.size() != 2)
        cerr << "found " << borders.size() << " borders for " << p << " out of " << points.size() << " border points" << endl;
}

