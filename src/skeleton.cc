#include "skeleton.hh"
#include <limits>
#include <boost/bind.hpp>
#include <algorithm>
#include "pool_allocator.hh"
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <iomanip>
#include <map>

using namespace boost;
using namespace std;
using namespace nav;

static const int VALUE_IGNORE               = 0;
static const int VALUE_SKELETON_VISITED     = 1;
static const int VALUE_SKELETON_NOT_VISITED = 2;

// Start value for marking of skeleton points.
//
// buildPlan() uses (VALUE_CORRIDORS_START + n) to mark that a skeleton point is
// part of corridor +n+, and then uses VALUE_CONNECTIONS_START (which is set to
// VALUE_CORRIDORS_START + corridors.size()) to mark the connection points.
static const int VALUE_CORRIDORS_START = 10;

void CorridorExtractionState::addBranch(PointID const& p, std::list<VoronoiPoint>& line)
{
    std::cerr << std::string(depth, ' ') << "adding branch at " << p << " of size " << line.size() << std::endl;
    displayLine(std::cerr, line);

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
    cerr << string(state.depth, ' ') << "visiting " << cur_point << endl;
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

void SkeletonExtraction::computeConnections(CorridorExtractionState& state)
{
    // First, a dead simple step: convert each branch into a corridor. When we
    // have done that, build the set of connection points by creating the
    // looking at the beginning and end point.
    //
    // We use the grid graph to check if there are places where a branch meets
    // the middle of another branch. We will post-process those later on
    list<Endpoint> endpoints;
    BranchMap::iterator branch_it = state.branches.begin();
    BranchMap::iterator const branch_end = state.branches.end();
    for (; branch_it != branch_end; ++branch_it)
    {
        // Keep a local copy to make the compiler potentially happier
        list<VoronoiPoint> line;
        line.swap(branch_it->second);

        // Remove all the voronoi points that have more than 2 borders
        while (!line.empty())
        {
            // the index of the corridor
            size_t corridor_idx = state.plan.corridors.size();

            PointID front_point = line.front().center, back_point;
            while (!line.empty() && line.front().borders.size() != 2)
                line.pop_front();

            Corridor::voronoi_iterator end_it   = line.begin();
            for (; end_it != line.end(); ++end_it)
            {
                back_point = end_it->center;
                state.graph.setValue(end_it->center.x, end_it->center.y, VALUE_CORRIDORS_START + corridor_idx);

                if (end_it->borders.size() != 2)
                {
                    cerr << corridor_idx << " " << end_it->borders.size() << " " << end_it->center << " " << *end_it << endl;
                    break;
                }
            }

            Corridor::voronoi_iterator cross_point_it = end_it;
            for (; cross_point_it != line.end(); ++cross_point_it)
            {
                if (cross_point_it->borders.size() == 2)
                    break;
            }
            if (cross_point_it == line.end())
                back_point = line.back().center;

            if (end_it != line.begin())
            {
                // register endpoints in +endpoints+, to create connections later on
                endpoints.push_back(Endpoint(front_point, corridor_idx, false));
                endpoints.push_back(Endpoint(back_point, corridor_idx, true));

                // Finally, create the new corridor
                Corridor& new_corridor = state.plan.newCorridor();
                new_corridor.voronoi.splice(new_corridor.voronoi.end(), line, line.begin(), end_it);
                new_corridor.update();
            }
        }
    }
    // OK, the branches set is now invalid (we spliced all the lines to the
    // corridors), so clear it
    state.branches.clear();

    int const VALUE_CONNECTIONS_START = VALUE_CORRIDORS_START + state.plan.corridors.size();

    // Register the "obvious" connection points, i.e. the ones that are exactly
    // at the same place
    vector< list<Endpoint> > connection_points;
    for (list<Endpoint>::const_iterator it = endpoints.begin();
            it != endpoints.end(); ++it)
    {
        Endpoint endp = *it;
        PointID p = endp.point;
        int current_endpoint = lround(state.graph.getValue(p.x, p.y));

        // If the value is greater than VALUE_ENDPOINT_START, it means we have
        // already an endpoint there.
        if (current_endpoint >= VALUE_CONNECTIONS_START)
            connection_points[current_endpoint - VALUE_CONNECTIONS_START].push_back(endp);
        else
        {
            int connection_index = connection_points.size();
            connection_points.push_back(list<Endpoint>());
            connection_points.back().push_back(endp);
            state.graph.setValue(p.x, p.y, connection_index + VALUE_CONNECTIONS_START);
        }
    }

    // Now merge the adjacent connection points. We also search for endpoints
    // that cut another branch in two halves.
    //
    // Note that we MUST NOT remove any element in connection_points. Instead,
    // we simply make the endpoint list empty to mark that this connection point
    // stopped being useful.

    // This map is used to register the potential splits. They are filtered out
    // later on
    typedef map<int, set<int> > SplitMap;
    SplitMap potential_splits;
    int connection_points_size = connection_points.size();
    for (int connection_idx = 0; connection_idx < connection_points_size; ++connection_idx)
    {
        list<Endpoint>& endpoints = connection_points[connection_idx];
        list<Endpoint>::iterator const endp_end = endpoints.end();
        for (list<Endpoint>::iterator endp_it = endpoints.begin(); endp_it != endp_end; ++endp_it)
        {
            PointID endp = endp_it->point;
            GridGraph::iterator n_it = state.graph.neighboursBegin(endp.x, endp.y);
            for (; !n_it.isEnd(); ++n_it)
            {
                int value = lround(n_it.getTargetValue());

                if (value >= VALUE_CONNECTIONS_START)
                { // this is another endpoint. Merge.
                    value -= VALUE_CONNECTIONS_START;

                    if (value != connection_idx)
                    {
                        SplitMap::iterator split_it = potential_splits.find(value);
                        if (split_it != potential_splits.end())
                        {
                            potential_splits[connection_idx].insert(
                                    split_it->second.begin(),
                                    split_it->second.end());
                            potential_splits.erase(split_it);
                        }
                        endpoints.splice(endpoints.end(), connection_points[value]);
                        n_it.setTargetValue(connection_idx + VALUE_CONNECTIONS_START);
                    }
                }
                else if (value >= VALUE_CORRIDORS_START)
                { // the target point is part of another corridor. Register as a
                  // possible link between an endpoint and the middle of a
                  // corridor
                    value -= VALUE_CORRIDORS_START;
                    potential_splits[connection_idx].insert(value);
                }
            }
        }
    }

    // Filter the connection that exists from the potential_splits mapping
    SplitMap::iterator split_it = potential_splits.begin();
    SplitMap::iterator const split_end = potential_splits.end();
    while (split_it != split_end)
    {
        int connection_idx = split_it->first;
        list<Endpoint> const& endpoints = connection_points[connection_idx];
        for (list<Endpoint>::const_iterator endp_it = endpoints.begin(); endp_it != endpoints.end(); ++endp_it)
            split_it->second.erase(endp_it->corridor_idx);

        if (split_it->second.empty())
            potential_splits.erase(split_it++);
        else
            ++split_it;
    }

    // Convert the connection point and split maps into the data structures that
    // CorridorExtractionState expects.
    //
    // We filter out the connections that invalid and/or are connected to
    // nothing (i.e. endpoints that are actually not connected)
    for (size_t connection_idx = 0; connection_idx != connection_points.size(); ++connection_idx)
    {
        list<Endpoint>& endpoints = connection_points[connection_idx];
        if (endpoints.empty())
            continue;

        SplitMap::iterator split_it =
            potential_splits.find(connection_idx);
        if (endpoints.size() == 1 && split_it == potential_splits.end())
        {
            // We still have to check if there is a split that involves this
            // corridor
            int corridor_idx = endpoints.front().corridor_idx;
            SplitMap::const_iterator split_it;
            for (split_it = potential_splits.begin(); split_it != potential_splits.end(); ++split_it)
            {
                if (split_it->second.count(corridor_idx))
                    break;
            }

            if (split_it == potential_splits.end())
            {
                cerr << "potential dead end: " << endpoints.front().point << " " << endpoints.front().corridor_idx << endl;
                state.simple_connectivity_corridors.push_back(endpoints.front().corridor_idx);
            }
            continue;
        }

        ConnectionPoints::iterator it =
            state.connection_points.insert(state.connection_points.end(), list<Endpoint>());
        it->swap(connection_points[connection_idx]);

        if (split_it != potential_splits.end())
        {
            SplitPoints::iterator state_split =
                state.split_points.insert(
                       state.split_points.end(),
                       make_pair(it, set<int>()) );
            state_split->second.swap(split_it->second);
        }
    }
}

pair<int, int> SkeletonExtraction::removeDeadEnds(CorridorExtractionState& state)
{
    int start_corridor = state.plan.findCorridorOf(state.plan.getStartPoint());
    int end_corridor   = state.plan.findCorridorOf(state.plan.getEndPoint());
    set<int> end_corridors;
    end_corridors.insert(start_corridor);
    end_corridors.insert(end_corridor);
    removeDeadEnds(state, end_corridors);
    return make_pair(start_corridor, end_corridor);
}

void SkeletonExtraction::removeDeadEnds(CorridorExtractionState& state, set<int> keepalive)
{
redo:
    size_t const start_size = state.plan.corridors.size();
    size_t i = 0;
    while (i < state.plan.corridors.size())
    {
        if (keepalive.count(i))
        {
            ++i;
            continue;
        }

        if (state.plan.corridors[i].isDeadEnd())
        {
            set<int> new_keepalive;
            for (set<int>::const_iterator it = keepalive.begin(); it != keepalive.end(); ++it)
            {
                if (*it > i) new_keepalive.insert(*it - 1);
                else new_keepalive.insert(*it);
            }
            new_keepalive.swap(keepalive);

            state.plan.removeCorridor(i);
        }
        else ++i;
    }

    if (start_size != state.plan.corridors.size())
        goto redo;
}

void SkeletonExtraction::registerConnections(CorridorExtractionState& state)
{
    // Register the connections we know already
    //
    // We do this before applying the split, as Plan::split will update the
    // connections for us
    for (ConnectionPoints::const_iterator conn_it = state.connection_points.begin();
            conn_it != state.connection_points.end(); ++conn_it)
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
        ConnectionPoints::iterator conn_it = split_it->first;
        PointID conn_p = conn_it->front().point;

        list< pair<int, int> > split_corridors;

        set<int> const& corridors = split_it->second;
        for (set<int>::const_iterator corridor_it = corridors.begin(); corridor_it != corridors.end(); ++corridor_it)
        {
            int corridor_idx = *corridor_it;
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
            cerr << "splitting " << corridor.name << " at " << split_point->center << " for " << conn_p << endl;
            Corridor& back_corridor = state.plan.split(corridor_idx, split_point);

            // Unfortunately, it is possible that the same corridor is split
            // elsewhere as well. We must check if that split will apply on the
            // front or back part of the updated corridor
            SplitPoints::iterator split_update_it = split_it;
            for (++split_update_it; split_update_it != split_end; ++split_update_it)
            {
                if (!split_update_it->second.count(corridor_idx))
                    continue;

                PointID p           = split_update_it->first->front().point;
                PointID front_match = corridor.findNearestMedian(p)->center;
                PointID back_match  = back_corridor.findNearestMedian(p)->center;
                if (front_match.distance2(p) > back_match.distance2(p))
                {
                    split_update_it->second.erase(corridor_idx);
                    split_update_it->second.insert(back_idx);
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
    removeDeadEnds(state);
    
    return state.plan;
}

//void SkeletonExtraction::buildPixelMap(Plan& result) const
//{
//    // Build the pixel-to-corridor map. In there, "0" means "multiple owners"
//    // (i.e. crossroads)
//    result.pixel_map.resize(width * height);
//    int const NO_OWNER = result.corridors.size();
//    int const MULTIPLE_OWNERS = 0;
//    fill(result.pixel_map.begin(), result.pixel_map.end(), NO_OWNER);
//    //cerr << "Pixel map (border)\n";
//    for (size_t corridor_idx = 1; corridor_idx < result.corridors.size(); ++corridor_idx)
//    {
//        MedianPoint::BorderList const& borders = result.corridors[corridor_idx].borders;
//
//        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
//            for (PointVector::const_iterator point = it->begin(); point != it->end(); ++point)
//            {
//                uint8_t& owner = result.pixel_map[point->x + point->y * width];
//                if (owner == NO_OWNER)
//                    owner = corridor_idx;
//                else
//                    owner = MULTIPLE_OWNERS;
//                //cerr << corridor_idx << " " << (int)owner << " " << point->x << " " << point->y << endl;
//            }
//    }
//
//    /** Now, take each pixel and assign its corridor based on its parents */
//    //cerr << "Pixel map (interior)\n";
//    for (int idx = 0; idx < width * height; ++idx)
//    {
//        if (result.pixel_map[idx] != NO_OWNER)
//            continue;
//
//        ParentMap::const_iterator borders_it = parents.find(const_cast<height_t*>(&heightmap[idx]));
//        if (borders_it == parents.end())
//            continue;
//
//        int owner = NO_OWNER;
//        MedianPoint::BorderList const& borders = borders_it->second.borders;
//        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
//            for (PointVector::const_iterator point = it->begin(); point != it->end(); ++point)
//            {
//                int border_owner = result.pixel_map[point->x + point->y * width];
//                if (owner == NO_OWNER)
//                {
//                    if (border_owner != MULTIPLE_OWNERS)
//                        owner = border_owner;
//                }
//                else if (border_owner != owner && border_owner != MULTIPLE_OWNERS)
//                    owner = MULTIPLE_OWNERS;
//                //cerr << (int)owner << " " << idx%width
//                    //<< " " << idx/width <<  " " << (int)border_owner << " " 
//                    //<< point->x << " " << point->y << endl;
//            }
//
//        result.pixel_map[idx] = owner;
//    }
//}

