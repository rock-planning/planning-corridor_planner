#include "skeleton.hh"
#include <limits>
#include <boost/bind.hpp>
#include <algorithm>
#include "pool_allocator.hh"
#include <boost/lexical_cast.hpp>

#include <iterator>
#include <iostream>
#include <iomanip>
#include <map>
#include <stdexcept>

#undef DEBUG
#ifdef DEBUG
#define DEBUG_OUT(x) cerr << x << endl;
#else
#define DEBUG_OUT(x)
#endif

using namespace boost;
using namespace std;
using namespace nav;

static const int VALUE_IGNORE               = 0;
static const int VALUE_SKELETON_VISITED     = 1;
static const int VALUE_SKELETON_NOT_VISITED = 2;

// Start value for marking of skeleton points.
//
// buildPlan() uses (VALUE_CORRIDORS_START + n) to mark that a skeleton point is
// part of corridor +n+, and then uses -crossroad_idx to mark the crossroads
static const int VALUE_CORRIDORS_START = 10;

void CorridorExtractionState::addBranch(PointID const& p, std::list<VoronoiPoint>& line)
{
#ifndef DEBUG
    DEBUG_OUT(std::string(depth, ' ') << "adding branch at " << p << " of size " << line.size());
    displayLine(cerr, line);
#endif

    BranchMap::iterator it = branches.insert( make_pair(p, list<VoronoiPoint>()) );
    it->second.swap(line);
}


SkeletonExtraction::SkeletonExtraction(size_t width, size_t height)
    : width(width)
    , height(height)
    , heightmap(width * height)
{
    fill(heightmap.begin(), heightmap.end(), 0);
}

SkeletonExtraction::~SkeletonExtraction()
{
}

PointID SkeletonExtraction::pointFromPtr(height_t const* ptr) const
{
    uint32_t offset = ptr - &heightmap[0];
    return PointID(offset % width, offset / width);
}

list<VoronoiPoint> SkeletonExtraction::process()
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
            DEBUG_OUT("taking " << pointFromPtr(c_ptr) << "(" << (void*)c_ptr << ")=" << (int)c_value);
            DEBUG_OUT("  with borders " << parents[c_ptr]);

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
                DEBUG_OUT( "  " << pointFromPtr(neighbour) << "(" << (void*)neighbour << ")=" << (int)*neighbour << " ~ " << propagated_value);
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
#ifndef DEBUG
                    cerr <<  "   ... found border ?" << parents[neighbour] << flush;
#endif
                    if (!parents[neighbour].isBorderAdjacent( parents[c_ptr] ))
                    {
                        DEBUG_OUT(" yes");

                        skeleton.insert( neighbour );
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else if (*neighbour == propagated_value)
                    {
                        DEBUG_OUT(" no");
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else
                    {
                        DEBUG_OUT(" no");
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

    list<VoronoiPoint> result;
    for (CandidateSet::const_iterator it = skeleton.begin(); it != skeleton.end(); ++it)
    {
        PointID p = pointFromPtr(*it);
        result.push_back(parents[*it]);
        VoronoiPoint& info = result.back();
        info.center = p;
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

bool SkeletonExtraction::isInside(int x, int y) const
{
    return heightmap[y * width + x] > 1;
}

pair<PointSet, PointSet> SkeletonExtraction::getBorderAndInside() const
{
    PointSet inside;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            if (isInside(x, y))
                inside.insert(PointID(x, y));
        }

    return make_pair(border, inside);
}

void SkeletonExtraction::initializeFromDStar(DStar const& dstar, int x, int y, float expand)
{
    typedef multimap<float, PointID, std::less<float> > OpenSet;
    OpenSet open_from_cost;
    map<PointID, OpenSet::iterator, std::less<PointID> > is_in_open;

    border.clear();
    parents.clear();
    fill(heightmap.begin(), heightmap.end(), 0);

    GridGraph const& nav_function = dstar.graph();

    float cost_max = nav_function.getValue(x, y) * expand;
    {
        PointID p0(x, y);
        OpenSet::iterator n_it = open_from_cost.insert( make_pair(0, p0) );
        is_in_open.insert( make_pair(p0, n_it) );
        heightmap[y * width + x] = MAX_DIST;
    }

    while(!open_from_cost.empty())
    {
        PointID point; float cost;
        tie(cost, point) = *open_from_cost.begin();
        open_from_cost.erase(open_from_cost.begin());
        is_in_open.erase(point);

        for (NeighbourConstIterator n = nav_function.neighboursBegin(point.x, point.y); !n.isEnd(); ++n)
        {
            float n_cost = cost + dstar.costOf(n);
            PointID n_p = PointID(n.x(), n.y());

            if (n_cost + n.getValue() <= cost_max)
            {
                if (isInside(n_p.x, n_p.y))
                {
                    map<PointID, OpenSet::iterator>::iterator open_it = is_in_open.find(n_p);
                    OpenSet::iterator n_it = open_it->second;
                    if (open_it != is_in_open.end() && n_it->first > n_cost)
                    {
                        open_from_cost.erase(open_it->second);
                        n_it = open_from_cost.insert( make_pair(n_cost, n_p) );
                        open_it->second = n_it;
                    }
                }
                else
                {
                    border.erase(n_p);
                    heightmap[n_p.y * width + n_p.x] = MAX_DIST;
                    OpenSet::iterator n_it = open_from_cost.insert( make_pair(n_cost, n_p) );
                    is_in_open.insert( make_pair(n_p, n_it) );
                }
            }
            else if (!isInside(n_p.x, n_p.y))
            {
                border.insert(n_p);
            }
        }
    }
}
list<VoronoiPoint> SkeletonExtraction::processDStar(DStar const& dstar, int x, int y, float expand)
{
    initializeFromDStar(dstar, x, y, expand);
    return process();
}

void SkeletonExtraction::initializeFromPointSets(PointSet const& inside, PointSet const& border)
{
    this->border = border;
    parents.clear();
    fill(heightmap.begin(), heightmap.end(), 0);
    for (PointSet::const_iterator it = inside.begin(); it != inside.end(); ++it)
        heightmap[it->x + it->y*width] = MAX_DIST;
}
list<VoronoiPoint> SkeletonExtraction::processPointSets(PointSet const& inside, PointSet const& border)
{
    initializeFromPointSets(inside, border);
    return process();
}

bool lineOrderingDFS(PointID const& cur_point, int neighbour_mask,
        list<VoronoiPoint>& parent_line,
        CorridorExtractionState& state)
{
    static const int TOP          = GridGraph::TOP;
    static const int TOP_LEFT     = GridGraph::TOP_LEFT;
    static const int TOP_RIGHT    = GridGraph::TOP_RIGHT;
    static const int RIGHT        = GridGraph::RIGHT;
    static const int LEFT         = GridGraph::LEFT;
    static const int BOTTOM       = GridGraph::BOTTOM;
    static const int BOTTOM_LEFT  = GridGraph::BOTTOM_LEFT;
    static const int BOTTOM_RIGHT = GridGraph::BOTTOM_RIGHT;
    static const int PROGRESSION_MASKS[8] = {
        RIGHT       | TOP_RIGHT | BOTTOM_RIGHT, // RIGHT
        RIGHT       | TOP_RIGHT | TOP,          // TOP_RIGHT
        TOP         | TOP_RIGHT | TOP_LEFT,     // TOP
        TOP         | LEFT      | TOP_LEFT,     // TOP_LEFT
        BOTTOM_LEFT | LEFT      | TOP_LEFT,     // LEFT
        BOTTOM_LEFT | LEFT      | BOTTOM,       // BOTTOM_LEFT
        BOTTOM_LEFT | BOTTOM    | BOTTOM_RIGHT, // BOTTOM
        RIGHT       | BOTTOM    | BOTTOM_RIGHT  // BOTTOM_RIGHT
    };

    bool detached  = true;
    bool branching = false;

    state.graph.setValue(cur_point.x, cur_point.y, VALUE_SKELETON_VISITED);
    VoronoiMap::iterator voronoi_it = state.voronoiMap.find(cur_point);
    DEBUG_OUT(string(state.depth, ' ') << "visiting " << cur_point);
    parent_line.push_back( *(voronoi_it->second) );
    state.voronoiMap.erase(voronoi_it);
    state.depth++;

    GridGraph::iterator n_it = state.graph.neighboursBegin(cur_point.x, cur_point.y, neighbour_mask);
    list<VoronoiPoint> result, cur_line;
    for (; !n_it.isEnd(); ++n_it)
    {
        int target_value = lround(n_it.getTargetValue());
        if (target_value != VALUE_SKELETON_NOT_VISITED) // either not actual point, or already visited
        {
            if (target_value == VALUE_SKELETON_VISITED)
                detached = false;
            continue;
        }

        PointID root = n_it.getTargetPoint();
        bool detached = lineOrderingDFS(n_it.getTargetPoint(), PROGRESSION_MASKS[n_it.getNeighbourIndex()],
                result, state);

        if (cur_line.empty())
            cur_line.swap(result);
        else if (!detached)
        {
            if (cur_line.size() < result.size())
                cur_line.swap(result);
        }
        else
        {
            branching = true;
            state.addBranch(cur_point, cur_line);
            cur_line.swap(result);
        }
    }
    
    state.depth--;
    if (!branching)
        parent_line.splice(parent_line.end(), cur_line);
    else if (!cur_line.empty())
        state.addBranch(cur_point, cur_line);

    return branching || detached;
}


void SkeletonExtraction::extractBranches(list<VoronoiPoint> const& points, CorridorExtractionState& state)
{
    // Initialize a GridGraph of the right size, and mark the voronoi points in
    // it. We will use it to traverse the terrain
    //
    // At the same time, keep a mapping from PointID to voronoi points. The
    // points will be removed from there as they are traversed.
    for (voronoi_const_iterator it = points.begin(); it != points.end(); ++it)
    {
        PointID center = it->center;
        state.voronoiMap.insert( make_pair(center, it) );
        state.graph.setValue(center.x, center.y, 2);
    }

    while (!state.voronoiMap.empty())
    {
        PointID start_point = state.front().center;
        list<VoronoiPoint> cur_line;
        lineOrderingDFS(start_point, GridGraph::DIR_ALL, cur_line, state);
        if (cur_line.size() > 1)
            state.addBranch( start_point, cur_line );
    }
}

struct GeometricEndpoint
{
    PointID p;
    Endpoint endp;
    GeometricEndpoint(PointID const& p, int idx, bool side)
        : p(p), endp(idx, side) {}
};

void SkeletonExtraction::computeConnections(CorridorExtractionState& state)
{
    // First, a dead simple step: convert each branch into a corridor. When we
    // have done that, build the set of connection points by creating the
    // looking at the beginning and end point.
    //
    // We use the grid graph to check if there are places where a branch meets
    // the middle of another branch. We will post-process those later on
    list< GeometricEndpoint > endpoints;
    set<PointID> crossroad_points;
    vector< set< Endpoint > > crossroads;

    BranchMap::iterator branch_it = state.branches.begin();
    BranchMap::iterator const branch_end = state.branches.end();
    for (; branch_it != branch_end; ++branch_it)
    {
        // Keep a local copy to make the compiler potentially happier
        list<VoronoiPoint> line;
        line.swap(branch_it->second);

#ifdef DEBUG
        cerr << "handling branch ";
        displayLine(cerr, line);
#endif

        // Remove all the voronoi points that have more than 2 borders
        while (!line.empty())
        {
            // the index of the corridor
            size_t corridor_idx = state.plan.corridors.size();

            if (line.front().borders.size() != 2)
            {
                int crossroad_idx = crossroads.size();
                crossroads.push_back( set<Endpoint>() );
                while (!line.empty() && line.front().borders.size() != 2)
                {
                    PointID p = line.front().center;
                    crossroad_points.insert(p);
                    state.graph.setValue(p.x, p.y, -crossroad_idx);
                    line.pop_front();
                }
                if (line.empty())
                    break;
            }

            Corridor::voronoi_iterator end_it = line.begin();
            VoronoiPoint const* last_point = 0;
            for (; end_it != line.end(); ++end_it)
            {
                state.graph.setValue(end_it->center.x, end_it->center.y, VALUE_CORRIDORS_START + corridor_idx);
                if (end_it->borders.size() != 2 || (last_point && !last_point->isBorderAdjacent(*end_it)))
                    break;

                last_point = &*end_it;
            }

            // Finally, create the new corridor
            Corridor& new_corridor = state.plan.newCorridor();
            new_corridor.voronoi.splice(new_corridor.voronoi.end(), line, line.begin(), end_it);
            DEBUG_OUT("corridor " << new_corridor.name << " " << new_corridor.frontPoint() << " => " << new_corridor.backPoint());
            new_corridor.update();

            // register endpoints in +endpoints+, to create connections later on
            endpoints.push_back(GeometricEndpoint(new_corridor.frontPoint(), corridor_idx, false));
            endpoints.push_back(GeometricEndpoint(new_corridor.backPoint(), corridor_idx, true));
        }
    }
    // OK, the branches set is now invalid (we spliced all the lines to the
    // corridors), so clear it
    state.branches.clear();

    // Get a copy of the graph, as we need to know by whom the endpoints are
    // owned.
    GridGraph ownerships(state.graph);

    // Register the "obvious" connection points, i.e. the ones that are exactly
    // at the same place
    for (list<GeometricEndpoint>::const_iterator it = endpoints.begin();
            it != endpoints.end(); ++it)
    {
        Endpoint endp = it->endp;
        PointID p     = it->p;
        crossroad_points.insert(p);

        int connection_index = crossroads.size();
        crossroads.push_back(set<Endpoint>());
        crossroads.back().insert(endp);
        state.graph.setValue(p.x, p.y, -connection_index);
    }

    // Now merge the adjacent connection points. We also search for endpoints
    // that cut another branch in two halves (registered in SplitMap below)
    //
    // Note that we MUST NOT remove any element in crossroads -- since we are
    // referring to the crossroads by their index. Instead, we simply make the
    // endpoint list empty to mark that this connection point stopped being
    // useful.
    //
    // SplitMap maps:
    //   crossroad_idx => (corridor_idx, point)
    //
    // to indicate that the given crossroad may split the given corridor at the
    // specified point
    typedef map<int, map< int, PointID> > SplitMap;
    SplitMap potential_splits;
    for (set<PointID>::const_iterator crossroad_it = crossroad_points.begin();
            crossroad_it != crossroad_points.end(); ++crossroad_it)
    {
        GridGraph::iterator n_it = state.graph.neighboursBegin(crossroad_it->x, crossroad_it->y);
        int owner_idx = lround(ownerships.getValue(crossroad_it->x, crossroad_it->y)) - VALUE_CORRIDORS_START;
        int crossroad_idx = lround(-n_it.getSourceValue());

        for (; !n_it.isEnd(); ++n_it)
        {
            int neighbour_owner_idx = lround(ownerships.getValue(n_it.x(), n_it.y())) - VALUE_CORRIDORS_START;
            if (owner_idx >= 0 && neighbour_owner_idx == owner_idx)
                continue;

            int pixel_owner = lround(n_it.getTargetValue());
            if (pixel_owner < 0)
            { // this is another crossroad. Merge.
                int neighbour_crossroad = -pixel_owner;
                if (neighbour_crossroad == crossroad_idx)
                    continue;

                SplitMap::iterator split_it = potential_splits.find(neighbour_crossroad);
                if (split_it != potential_splits.end())
                {
                    potential_splits[crossroad_idx].insert(
                            split_it->second.begin(),
                            split_it->second.end());
                    potential_splits.erase(split_it);
                }

                n_it.setTargetValue(-crossroad_idx);
                set<Endpoint>& points = crossroads[neighbour_crossroad];
                crossroads[crossroad_idx].insert(points.begin(), points.end());
                // we clear to mark that this crossroad does exist anymore
                // See the big warning at the beginning of this code section
                points.clear();
            }
            else if (pixel_owner >= VALUE_CORRIDORS_START)
            { // the target point is part of another corridor. Register as a
                // possible link between an endpoint and the middle of a
                // corridor
                potential_splits[crossroad_idx][pixel_owner - VALUE_CORRIDORS_START] = *crossroad_it;
            }
        }
    }

    // Filter the connection that exists from the potential_splits mapping
    //
    // More specifically, if a connection potentially splits a set of corridor,
    // we check that the "splitted" corridors aren't already registered in the
    // connection itself ... which would mean that it is a false positive.
    SplitMap::iterator split_it = potential_splits.begin();
    SplitMap::iterator const split_end = potential_splits.end();
    while (split_it != split_end)
    {
        int crossroad_idx = split_it->first;
        set<Endpoint> const& endpoints = crossroads[crossroad_idx];
        map<int, PointID>& splitted_corridors = split_it->second;
        for (set<Endpoint>::const_iterator endp_it = endpoints.begin(); endp_it != endpoints.end(); ++endp_it)
            splitted_corridors.erase(endp_it->corridor_idx);

        if (splitted_corridors.empty())
            potential_splits.erase(split_it++);
        else
        {
            vector<int> corridors;
            for (map<int, PointID>::const_iterator it = splitted_corridors.begin();
                    it != splitted_corridors.end(); ++it)
                corridors.push_back(it->first);
            vector<int> in_corridors;
            transform(endpoints.begin(), endpoints.end(), back_inserter(in_corridors),
                    bind(&Endpoint::corridor_idx, _1));

#ifdef DEBUG
            cerr << "potential split of ";
            copy(corridors.begin(), corridors.end(), ostream_iterator<int>(cerr, ", "));
            cerr << " by ";
            copy(in_corridors.begin(), in_corridors.end(), ostream_iterator<int>(cerr, ", "));
            cerr << endl;
#endif
            ++split_it;
        }
    }

    // Convert the connection point and split maps into the data structures that
    // CorridorExtractionState expects.
    //
    // We filter out the connections that invalid and/or are connected to
    // nothing (i.e. endpoints that are actually not connected)
    for (size_t connection_idx = 0; connection_idx != crossroads.size(); ++connection_idx)
    {
        set<Endpoint>& endpoints = crossroads[connection_idx];
        if (endpoints.size() < 2)
            continue;

        Crossroads::iterator it =
            state.crossroads.insert(state.crossroads.end(), list<Endpoint>());
        it->insert(it->end(), endpoints.begin(), endpoints.end());

        SplitMap::iterator split_it =
            potential_splits.find(connection_idx);
        if (split_it != potential_splits.end())
        {
            SplitPoints::iterator state_split =
                state.split_points.insert(
                       state.split_points.end(),
                       make_pair(it, map<int, PointID>()) );
            state_split->second.swap(split_it->second);
        }
    }
}

void SkeletonExtraction::registerConnections(CorridorExtractionState& state)
{
    // Register the connections we know already
    //
    // We do this before applying the split, as Plan::split will update the
    // connections for us
    for (Crossroads::const_iterator conn_it = state.crossroads.begin();
            conn_it != state.crossroads.end(); ++conn_it)
    {
        list<Endpoint>::const_iterator const conn_end = conn_it->end();

        list<Endpoint>::const_iterator source_it = conn_it->begin();
        for (; source_it != conn_end; ++source_it)
        {
            Corridor& source = state.plan.corridors[source_it->corridor_idx];

            list<Endpoint>::const_iterator target_it = conn_it->begin();
            for (; target_it != conn_end; ++target_it)
            {
                if (target_it->corridor_idx == source_it->corridor_idx) continue;
                source.addConnection(source_it->side, target_it->corridor_idx, target_it->side);
            }
        }
    }
}

void SkeletonExtraction::applySplits(CorridorExtractionState& state)
{
    // Apply the splits that +computeConnections+ has detected
    SplitPoints::iterator split_it = state.split_points.begin();
    SplitPoints::iterator const split_end = state.split_points.end();
    for (; split_it != split_end; ++split_it)
    {
        Crossroads::iterator conn_it = split_it->first;
        list< pair<int, int> > split_corridors;

        map<int, PointID> const& corridors = split_it->second;
        for (map<int, PointID>::const_iterator corridor_it = corridors.begin();
                corridor_it != corridors.end(); ++corridor_it)
        {
            int corridor_idx = corridor_it->first;
            PointID conn_p   = corridor_it->second;
            Corridor& corridor = state.plan.corridors[corridor_idx];
            Corridor::voronoi_iterator split_point =
                corridor.findNearestMedian(conn_p);

            if (split_point == corridor.voronoi.begin())
            {
                split_corridors.push_back( make_pair(-1, corridor_idx) );
                continue;
            }
            else if (split_point == (--corridor.voronoi.end()))
            {
                split_corridors.push_back( make_pair(corridor_idx, -1) );
                continue;
            }


            size_t back_idx = state.plan.corridors.size();
            DEBUG_OUT("splitting " << corridor.name << " at " << split_point->center << " for " << conn_p);
            Corridor& back_corridor = state.plan.split(corridor_idx, split_point);

            // Unfortunately, it is possible that the same corridor is split
            // elsewhere as well. We must check if that split will apply on the
            // front or back part of the updated corridor
            SplitPoints::iterator split_update_it = split_it;
            for (++split_update_it; split_update_it != split_end; ++split_update_it)
            {
                map<int, PointID>::iterator other_split =
                    split_update_it->second.find(corridor_idx);
                if (other_split == split_update_it->second.end())
                    continue;

                PointID p           = other_split->second;
                PointID front_match = corridor.findNearestMedian(p)->center;
                PointID back_match  = back_corridor.findNearestMedian(p)->center;
                if (front_match.distance2(p) > back_match.distance2(p))
                {
                    split_update_it->second.erase(other_split);
                    split_update_it->second.insert(make_pair(back_idx, p));
                }
            }

            split_corridors.push_back(make_pair(corridor_idx, back_idx));
        }

        // Create a two way connection between all the points in *conn_it and
        // each of the corridors in split_corridors
        list<Endpoint>::const_iterator const conn_end = conn_it->end();
        list<Endpoint>::const_iterator endp_it = conn_it->begin();
        for (endp_it = conn_it->begin(); endp_it != conn_end; ++endp_it)
        {
            int   endp_idx = endp_it->corridor_idx;
            bool endp_side = endp_it->side;
            Corridor& endp_corridor = state.plan.corridors[endp_idx];

            for (list< pair<int, int> >::const_iterator split_result_it = split_corridors.begin();
                    split_result_it != split_corridors.end(); ++split_result_it)
            {
                int split_front_idx = split_result_it->first;
                int split_back_idx = split_result_it->first;

                Corridor* split_front = NULL;
                if (split_front_idx != -1)
                {
                    split_front = &state.plan.corridors[split_front_idx];
                    endp_corridor.addConnection(endp_side, split_front_idx, true);
                    split_front->addConnection(true, endp_idx, endp_side);
                }

                Corridor* split_back = NULL;
                if (split_back_idx != -1)
                {
                    split_back = &state.plan.corridors[split_back_idx];
                    endp_corridor.addConnection(endp_side, split_back_idx, false);
                    split_back->addConnection(false, endp_idx, endp_side);
                }

                // First, add connections between front and back as Plan::split
                // does not do that
                if (split_front && split_back)
                {
                    split_back->addConnection(false, split_front_idx, true);
                    split_front->addConnection(true, split_back_idx, false);
                }
            }
        }
    }
}

Plan SkeletonExtraction::buildPlan(PointID const& start_point, PointID const& end_point, GridGraph const& nav_function,
        std::list<VoronoiPoint> const& points)
{
    CorridorExtractionState state(start_point, end_point, nav_function);

    extractBranches(points, state);
    computeConnections(state);

    registerConnections(state);
    applySplits(state);

    state.plan.createEndpointCorridor(state.plan.getStartPoint(), false);
    state.plan.createEndpointCorridor(state.plan.getEndPoint(), true);

    return state.plan;
}

