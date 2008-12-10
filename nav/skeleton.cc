#include "skeleton.hh"
#include <limits>
#include <boost/bind.hpp>
#include <algorithm>

#include <iostream>
#include <iomanip>

using boost::bind;
using boost::mem_fn;
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
    std::map<height_t*, MedianPoint > parents;
    for (PointSet::const_iterator it = border.begin(); it != border.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
        {
            height_t* c_ptr = &heightmap[it->y * width + it->x];
            *c_ptr = 1;
            candidates.insert( c_ptr );
            parents[c_ptr].addBorderPoint( pointFromPtr(c_ptr) );
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
            cerr << "  with borders " << parents[c_ptr] << endl;

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
                    cerr <<  "   ... found border ?" << parents[neighbour] << flush;
                    if (!parents[neighbour].isAdjacent( parents[c_ptr] ))
                    {
                        cerr << " yes" << endl;

                        skeleton.insert( neighbour );
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else if (*neighbour == propagated_value)
                    {
                        cerr << " no" << endl;
                        parents[neighbour].mergeBorders( parents[c_ptr] );
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
        info = parents[*it];
        info.distance = **it;
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
    return line;
}

list<Corridor> SkeletonExtraction::buildGraph(MedianLine const& points)
{
    typedef list<Corridor> CorridorList;
    CorridorList corridors;

    for (MedianLine::const_iterator p = points.begin(); p != points.end(); ++p)
    {
        if (p->second.borders.size() > 2)
            continue;

        // Search for one element in \c corridors that is adjacent to \c p (per
        // the meaning of MedianPoint::isAdjacent, see documentation).
        //
        // If there is one, merge that point into the corresponding corridor.
        // Otherwise, create a new corridor.
        CorridorList::iterator adjacent_corridor =
            find_if(corridors.begin(), corridors.end(),
                    bind( logical_and<bool>(),
                        bind(mem_fn(&MedianPoint::isAdjacent), _1, p->second),
                        bind(mem_fn(&Corridor::isNeighbour), _1, p->first)));

        if (adjacent_corridor == corridors.end())
        {
            Corridor new_corridor;
            new_corridor.add(*p);
            corridors.push_back(new_corridor);
            cerr << "new corridor for " << p->first << " " << p->second << endl;
        }
        else
        {
            // If we found an adjacent corridor, then the new point could
            // actually connect two previously non-adjacent corridors.
            // Merge them if it is the case.
            CorridorList::iterator merge_into = adjacent_corridor++;
            cerr << "adding " << p->first << " " << p->second << endl;
            merge_into->add(*p);
            while (adjacent_corridor != corridors.end())
            {
                if (adjacent_corridor->isAdjacent(p->second) && adjacent_corridor->isNeighbour(p->first))
                {
                    cerr << "merging" << endl;
                    merge_into->merge(*adjacent_corridor);
                    corridors.erase(adjacent_corridor++);
                }
                else
                    ++adjacent_corridor;
            }
        }
    }
    return corridors;
}

