#include "skeleton.hh"
#include <limits>
#include <boost/bind.hpp>
#include <algorithm>

#include <iostream>
#include <iomanip>

using namespace boost;
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
    parents.clear();
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

// displayHeightMap(cerr);

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
            // cerr << "taking " << pointFromPtr(c_ptr) << "(" << (void*)c_ptr << ")=" << (int)c_value << endl;
            // cerr << "  with borders " << parents[c_ptr] << endl;

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
                //cerr <<  "  " << pointFromPtr(neighbour) << "(" << (void*)neighbour << ")=" << (int)*neighbour << " ~ " << propagated_value << endl;
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
                    //cerr <<  "   ... found border ?" << parents[neighbour] << flush;
                    if (!parents[neighbour].isBorderAdjacent( parents[c_ptr] ))
                    {
                        //cerr << " yes" << endl;

                        skeleton.insert( neighbour );
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else if (*neighbour == propagated_value)
                    {
                        //cerr << " no" << endl;
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else
                    {
                        //cerr << " no" << endl;
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

    //displayHeightMap(cerr);

    MedianLine result;
    for (CandidateSet::const_iterator it = skeleton.begin(); it != skeleton.end(); ++it)
    {
        PointID p = pointFromPtr(*it);
        MedianPoint& info = result[p];
        info = parents[*it];
        info.width = **it;
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

void SkeletonExtraction::registerConnections(PointID source_point, int source_idx, map<PointID, int> const& targets, vector<Corridor>& corridors)
{
    Corridor& source = corridors[source_idx];
    for (map<PointID, int>::const_iterator target_it = targets.begin(); target_it != targets.end(); ++target_it)
    {
        PointID target_point = target_it->first;
        int target_idx = target_it->second;
        Corridor& target = corridors[target_idx];

        source.connections.push_back( make_tuple(source_point, target_idx, target_point));
        target.connections.push_back( make_tuple(target_point, source_idx, source_point));
    }
}

Plan SkeletonExtraction::buildPlan(MedianLine points)
{
    Plan result;
    result.width = width;
    result.height = height;
    vector<Corridor>& corridors = result.corridors;
    corridors.push_back(Corridor());

    // This registers the connections that we know about from points in the map
    // that are not yet in the corridor set and existing corridors.
    typedef map<PointID, map<PointID, int> > ConnectionMap;
    ConnectionMap connections;
    ConnectionMap in_out;

    // Move all points that have a connectivity of more than 2 to a separate
    // set. This set will be used to finalize the connections (but they won't be
    // part of any corridors).
    list<PointSet> crossroads;
    for (MedianLine::iterator it = points.begin(); it != points.end(); )
    {
        if (it->second.borders.size() > 2)
        {
            updateConnectedSets(crossroads, it->first);
            points.erase(it++);
        }
        else ++it;
    }

    // To build the corridor set, we consider a point in the point set and
    // insert it as a seed in the +propagation+ set. Then, we do the following:
    //  
    //   while !propagation.empty?
    //     take p out of propagation
    //     add in propagation the points P for which border(P) is neighbouring border(p)
    //   end
    while (! points.empty())
    {
        PointID seed = points.begin()->first;

        corridors.push_back(Corridor());
        Corridor& corridor = corridors.back();
        int corridor_idx = corridors.size() - 1;

        PointSet propagation;
        propagation.insert(seed);
        while (!propagation.empty())
        {
            PointID p = *propagation.begin();
            corridor.add(p, points[p]);

            // Find out if this point is connecting the corridor being built to
            // an already defined corridor. This creates a symmetric connection
            // between the two objects.
            ConnectionMap::iterator p_conn = connections.find(p);
            if (p_conn != connections.end())
                registerConnections(corridor_idx, p_conn, corridors);

            points.erase(p);
            propagation.erase(propagation.begin());

            for (int dy = -1; dy < 2; ++dy)
            {
                for (int dx = -1; dx < 2; ++dx)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    PointID neighbour(p.x + dx, p.y + dy);
                    if (heightmap[neighbour.x + neighbour.y * width] == 0)
                    { // this is a way to leave the zone. Save it, it will be
                      // registered as a connection to 0 afterwards.
                        in_out[neighbour][p] = corridor_idx;
                        continue;
                    }

                    MedianLine::iterator neighbour_it = points.find(neighbour);
                    if (neighbour_it == points.end())
                        continue;

                    if (corridor.isBorderAdjacent(neighbour_it->second))
                        propagation.insert(neighbour_it->first);
                    else
                    {
                        if (!connections[p].count(neighbour_it->first))
                            connections[neighbour_it->first][p] = corridor_idx;
                    }
                }
            }
        }
    }

    map< PointID, int > connected_corridors;
    for (list<PointSet>::const_iterator it = crossroads.begin(); it != crossroads.end(); ++it)
    {
        connected_corridors.clear();
        for (size_t i = 1; i < corridors.size(); ++i)
        {
            Corridor& corridor = corridors[i];
            PointSet::const_iterator crossroad_point = find_if(it->begin(), it->end(), bind(&Corridor::isMedianNeighbour, ref(corridor), _1));
            if (crossroad_point != it->end())
            {
                PointID endpoint = corridor.adjacentEndpoint(*crossroad_point);
                registerConnections(endpoint, i, connected_corridors, corridors);
                connected_corridors.insert( make_pair(endpoint, i) );
            }
        }
    }

    for (ConnectionMap::const_iterator it = in_out.begin(); it != in_out.end(); ++it)
        registerConnections(0, it, corridors);

    return result;
}

void SkeletonExtraction::buildPixelMap(Plan& result) const
{
    // Build the pixel-to-corridor map. In there, "0" means "multiple owners"
    // (i.e. crossroads)
    result.pixel_map.resize(width * height);
    int const NO_OWNER = result.corridors.size();
    int const MULTIPLE_OWNERS = 0;
    fill(result.pixel_map.begin(), result.pixel_map.end(), NO_OWNER);
    //cerr << "Pixel map (border)\n";
    for (size_t corridor_idx = 1; corridor_idx < result.corridors.size(); ++corridor_idx)
    {
        MedianPoint::BorderList const& borders = result.corridors[corridor_idx].borders;

        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
            for (PointSet::const_iterator point = it->begin(); point != it->end(); ++point)
            {
                uint8_t& owner = result.pixel_map[point->x + point->y * width];
                if (owner == NO_OWNER)
                    owner = corridor_idx;
                else
                    owner = MULTIPLE_OWNERS;
                //cerr << corridor_idx << " " << (int)owner << " " << point->x << " " << point->y << endl;
            }
    }

    /** Now, take each pixel and assign its corridor based on its parents */
    //cerr << "Pixel map (interior)\n";
    for (int idx = 0; idx < width * height; ++idx)
    {
        if (result.pixel_map[idx] != NO_OWNER)
            continue;

        ParentMap::const_iterator borders_it = parents.find(const_cast<height_t*>(&heightmap[idx]));
        if (borders_it == parents.end())
            continue;

        int owner = NO_OWNER;
        MedianPoint::BorderList const& borders = borders_it->second.borders;
        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
            for (PointSet::const_iterator point = it->begin(); point != it->end(); ++point)
            {
                int border_owner = result.pixel_map[point->x + point->y * width];
                if (owner == NO_OWNER)
                {
                    if (border_owner != MULTIPLE_OWNERS)
                        owner = border_owner;
                }
                else if (border_owner != owner && border_owner != MULTIPLE_OWNERS)
                    owner = MULTIPLE_OWNERS;
                //cerr << (int)owner << " " << idx%width
                    //<< " " << idx/width <<  " " << (int)border_owner << " " 
                    //<< point->x << " " << point->y << endl;
            }

        result.pixel_map[idx] = owner;
    }
}

