#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <iterator>
#include "skeleton.hh"
#include <queue>
#include <boost/bind.hpp>
#include <boost/function.hpp>

// #include <CGAL/Cartesian.h>
// #include <CGAL/convex_hull_2.h>

#define DEBUG
#ifdef DEBUG
#define DEBUG_OUT(x) cerr << x << endl;
#else
#define DEBUG_OUT(x)
#endif

static const int ENDPOINT_UNKNOWN = corridor_planner::Corridor::ENDPOINT_UNKNOWN;
static const int ENDPOINT_BACK   = corridor_planner::Corridor::ENDPOINT_BACK;
static const int ENDPOINT_FRONT    = corridor_planner::Corridor::ENDPOINT_FRONT;
static const int ENDPOINT_BIDIR   = corridor_planner::Corridor::ENDPOINT_BIDIR;

using namespace std;
using namespace corridor_planner;
using namespace boost;

Plan::Plan() {}
Plan::Plan(PointID start, PointID end, GridGraph const& nav_function)
    : m_start(start), m_end(end), m_nav_function(nav_function)
    , m_corridor_names(0) {}

PointID Plan::getStartPoint() const { return m_start; }
PointID Plan::getEndPoint() const { return m_end; }
GridGraph const& Plan::getNavigationFunction() const { return m_nav_function; }

void Plan::setStartPoint(PointID const& p) { m_start = p; }
void Plan::setEndPoint(PointID const& p) { m_end = p; }
void Plan::setNavigationFunction(GridGraph const& nav_function)
{ m_nav_function = nav_function; }

void Plan::clear()
{ corridors.clear(); }

Corridor& Plan::newCorridor()
{
    corridors.push_back(Corridor());
    Corridor& result = corridors.back();
    result.name = boost::lexical_cast<string>(++m_corridor_names);
    return result;
}

void Plan::removeCorridor(int idx)
{
    for (size_t i = idx + 1; i < corridors.size(); ++i)
        corridors[i].swap(corridors[i - 1]);

    corridors.resize(corridors.size() - 1);
    for (corridor_iterator corridor = corridors.begin(); corridor != corridors.end(); ++corridor)
    {
        Corridor::Connections& connections = corridor->connections;
        Corridor::connection_iterator it = connections.begin();
        while (it != connections.end())
        {
            int const target_idx = it->target_idx;
            if (target_idx == idx)
                connections.erase(it++);
            else
            {
                if (target_idx > idx)
                    it->target_idx--;

                ++it;
            }
        }
    }
}

void Plan::removeCorridors(vector<bool> const& to_remove)
{
    for (unsigned int i = 0; i < to_remove.size(); ++i)
    {
        int idx = to_remove.size() - 1 - i;
        if (to_remove[idx])
            removeCorridor(idx);
    }
}

void Plan::removeCorridors(set<int> const& to_remove)
{
    for_each(to_remove.rbegin(), to_remove.rend(),
            bind(&Plan::removeCorridor, this, _1));
}

void Plan::concat(Plan const& other)
{
    int merge_start = corridors.size();
    copy(other.corridors.begin(), other.corridors.end(), back_inserter(corridors));

    for (vector<Corridor>::iterator it = corridors.begin() + merge_start; it != corridors.end(); ++it)
        for (Corridor::connection_iterator c = it->connections.begin(); c != it->connections.end(); ++c)
            c->target_idx += merge_start;
}

void Plan::moveConnections(size_t from_idx, size_t into_idx)
{
    corridors[from_idx].removeConnectionsTo(into_idx);
    corridors[into_idx].moveOutgoingConnections(corridors[from_idx]);
    corridors[into_idx].removeConnectionsTo(from_idx);

    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (i == into_idx || i == from_idx)
            continue;

        corridors[i].moveConnections(from_idx, into_idx);
    }
}

Corridor& Plan::split(int corridor_idx, Corridor::voronoi_iterator it)
{
    Corridor& front_corridor = corridors[corridor_idx];

    Corridor back_corridor;

    // TODO: better bounding boxes
    back_corridor.bbox        = front_corridor.bbox;
    back_corridor.median_bbox = front_corridor.median_bbox;

    back_corridor.voronoi.splice(back_corridor.end(),
            front_corridor.voronoi, it, front_corridor.voronoi.end());

    // Check that both result corridors are not empty
    if (back_corridor.voronoi.empty())
        throw std::runtime_error("split() leads to an empty front corridor");
    if (front_corridor.voronoi.empty())
        throw std::runtime_error("split() leads to an empty back corridor");

    // Take one element of a border in +it+, find in which corridor border it
    // is, and split the boundary at that point
    //
    // Repeat for the other border.
    VoronoiPoint const& last_corr_point      = front_corridor.voronoi.back();
    {
        VoronoiPoint::BorderList::const_iterator border_it = last_corr_point.borders.begin();
        VoronoiPoint::BorderList::const_iterator const border_end = last_corr_point.borders.end();

        for (; border_it != border_end; ++border_it)
        {
            if (border_it->empty())
                throw std::logic_error("empty border found");

            PointID p = border_it->front();

	    list<PointID>::iterator boundary_it[2];
	    boundary_it[0] =
		findNearest(front_corridor.boundaries[0].begin(), front_corridor.boundaries[0].end(), p);
	    boundary_it[1] =
		findNearest(front_corridor.boundaries[1].begin(), front_corridor.boundaries[1].end(), p);

	    double d0 = boundary_it[0]->distance2(p);
	    double d1 = boundary_it[1]->distance2(p);
	    int boundary_index = (d0 < d1) ? 0 : 1;

	    back_corridor.boundaries[boundary_index].splice(
		    back_corridor.boundaries[boundary_index].end(),
		    front_corridor.boundaries[boundary_index],
		    boundary_it[boundary_index],
		    front_corridor.boundaries[boundary_index].end());
	    DEBUG_OUT("   splitting boundary " << boundary_index << " at " << *boundary_it[boundary_index] << " for " << p);
        }
    }

    // Update the connections that go to +corridor_idx+
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (i == (size_t)corridor_idx)
            continue;

        Corridor::Connections& connections = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            if (conn_it->target_idx == corridor_idx && conn_it->target_side == true)
                conn_it->target_idx = corridors.size();
        }
    }

    // Update the connections which come from +corridor_idx+
    Corridor::Connections& connections = front_corridor.connections;
    Corridor::Connections::iterator conn_it = connections.begin();
    while (conn_it != connections.end())
    {
        if (conn_it->this_side == true)
        {
            back_corridor.addConnection(conn_it->this_side, conn_it->target_idx, conn_it->target_side);
            connections.erase(conn_it++);
        }
        else ++conn_it;
    }

    corridors.push_back(back_corridor);
    corridors.back().name = front_corridor.name + "/2";
    front_corridor.name += "/1";
    return corridors.back();
}

void Plan::createEndpointCorridor(PointID const& endpoint, bool is_end)
{
    int current_idx = findCorridorOf(endpoint);
    Corridor& current_corridor = corridors[current_idx];
    bool endp_side = (is_end ? false : true);

    std::string name = current_corridor.name + "_" + (is_end ? "end" : "start");
    Corridor  endp_corridor = Corridor::singleton(endpoint, name);
    if (is_end)
        endp_corridor.end_types[0] = ENDPOINT_FRONT;
    else
        endp_corridor.end_types[0] = ENDPOINT_BACK;


    Corridor::voronoi_iterator it = current_corridor.findNearestMedian(endpoint);

    if (it == current_corridor.begin() || it == (--current_corridor.end()))
    { // The end point does not split a corridor in two, just connect
      // endp_corridor to the beginning of current_corridor
        int  endp_idx = corridors.size();
        bool attach_side = !(it == current_corridor.begin());
        for (Corridor::connection_iterator conn_it = current_corridor.connections.begin();
                conn_it != current_corridor.connections.end(); ++conn_it)
        {
            if (conn_it->this_side == attach_side)
            {
                endp_corridor.addConnection(endp_side, conn_it->target_idx, conn_it->target_side);
                corridors[conn_it->target_idx].addConnection(conn_it->target_side, endp_idx, endp_side);
            }
        }
        current_corridor.addConnection(attach_side, endp_idx, endp_side);
        endp_corridor.addConnection(endp_side, current_idx, attach_side);
        corridors.push_back(endp_corridor);
        DEBUG_OUT(endp_corridor.connections.size());
        DEBUG_OUT("attached " << endp_corridor.name << " at the " << (attach_side ? "back" : "front") << " of " << current_corridor.name);
        return;
    }

    // We need to split +current_corridor+ in two
    DEBUG_OUT("splitting " << current_corridor.name << " to attach " << endp_corridor.name);

    split(current_idx, it);

    int front_idx  = current_idx;
    Corridor& front_corridor = corridors[front_idx];
    int back_idx = corridors.size() - 1;
    Corridor& back_corridor = corridors[back_idx];

    front_corridor.checkConsistency();
    back_corridor.checkConsistency();

    // WARN: endp_idx must be initialized here, as split() adds a new
    // WARN: corridor to the corridor set, and therefore changes the index
    // WARN: of the endpoint corridor !
    int endp_idx = corridors.size();
    endp_corridor.addConnection(endp_side, front_idx, true);
    front_corridor.addConnection(true, endp_idx, endp_side);
    endp_corridor.addConnection(endp_side, back_idx, false);
    back_corridor.addConnection(false, endp_idx, endp_side);

    corridors.push_back(endp_corridor);

    if (front_corridor.isSingleton())
    {
        DEBUG_OUT("removing " << front_corridor.name);
        moveConnections(front_idx, back_idx);
        if (back_corridor.isSingleton())
            throw std::logic_error("both corridors are singletons");
        removeCorridor(front_idx);
    }
    else if (back_corridor.isSingleton())
    {
        DEBUG_OUT("removing " << back_corridor.name);
        moveConnections(back_idx, front_idx);
        removeCorridor(back_idx);
    }
}

void Plan::simplify(double margin_factor, int min_width)
{
    DEBUG_OUT("");
    DEBUG_OUT("");
    DEBUG_OUT("");
    DEBUG_OUT("checking consistency before simplification");
    checkConsistency();

    std::vector<bool> to_remove(corridors.size(), false);
    std::vector<Path> all_paths;
    removeBackToBackConnections(margin_factor);
    removeDeadEnds();
    mergeSimpleCrossroads_directed();

    for (int pass = 0; pass < 2; ++pass)
    {
        bool did_something = true;
        while (did_something)
        {
            unsigned long last_count = corridors.size();
            removeUselessCorridorConnections();
            removeDeadEnds();
            mergeSimpleCrossroads_directed();
            did_something = (last_count != corridors.size());
        }

        if (min_width == 0)
            break;

        all_paths.clear();
        fill(to_remove.begin(), to_remove.end(), false);
        computeAllPaths(all_paths);
        filterDeadEndPaths(all_paths);
        removeNarrowCorridors(all_paths, min_width, to_remove);
        removeCorridors(to_remove);
    }

    for (size_t i = 0; i < corridors.size(); ++i)
        corridors[i].update();

    checkConsistency();
}

void Plan::computeAllPaths(std::vector<Path>& all_paths) const
{
    Path current_path;
    int start_corridor = findStartCorridor();
    bool start_side = false;
    std::vector<int> stack;
    stack.resize(corridors.size(), 0);
    computeAllPaths(all_paths, current_path, start_corridor, start_side, stack);
}

void Plan::computeAllPaths(std::vector<Path>& all_paths, Path& current_path, int next_path, int in_side, std::vector<int>& stack) const
{
    current_path.push_back( std::make_pair(next_path, in_side) );
    typedef Corridor::Connections Connections;
    Connections const& connections = corridors[next_path].connections;

    bool added = false;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->this_side == in_side)
            continue;

        int target_idx = it->target_idx;
        int target_side = it->target_side ? 1 : 0;
        if (stack[target_idx] & (target_side + 1))
            continue;

        added = true;
        stack[target_idx] |= (target_side + 1);
        computeAllPaths(all_paths, current_path, target_idx, target_side, stack);
    }

    if (!added)
        all_paths.push_back(current_path);

    current_path.pop_back();
}

static bool widerCorridorComparison(Plan const& plan, int a_idx, int b_idx)
{
    Corridor const& a = plan.corridors[a_idx];
    Corridor const& b = plan.corridors[b_idx];
    return a.min_width > b.min_width;
}

void Plan::removeNarrowCorridors(std::vector<Path>& all_paths, double min_width, std::vector<bool>& to_delete)
{
    std::priority_queue<int, std::vector<int>, boost::function<bool (int, int)> >
        candidates(boost::bind(widerCorridorComparison, boost::ref(*this), _1, _2));
    for (unsigned int corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        if (!to_delete[corridor_idx] && corridors[corridor_idx].min_width < min_width)
            candidates.push(corridor_idx);
    }

    std::vector<unsigned int> corridor_use_count;
    corridor_use_count.resize(corridors.size());
    std::vector<bool> critical_paths;
    critical_paths.resize(corridors.size());

    while (!candidates.empty())
    {
        // Compute the use count for all corridors. In the process, remove paths
        // that have deleted corridors
        std::fill(corridor_use_count.begin(), corridor_use_count.end(), 0);
        for (unsigned int path_idx = 0; path_idx < all_paths.size(); ++path_idx)
        {
            Path& path = all_paths[path_idx];
            for (unsigned int i = 0; i < path.size(); ++i)
                corridor_use_count[path[i].first]++;
        }

        for (unsigned int i = 0; i < corridors.size(); ++i)
            critical_paths[i] = (corridor_use_count[i] == all_paths.size());

        int corridor_idx = candidates.top();
        candidates.pop();

        if (to_delete[corridor_idx] || critical_paths[corridor_idx])
            continue;

        to_delete[corridor_idx] = true;
        filterDeletedPaths(all_paths, to_delete);
    }
}

void Plan::filterDeletedPaths(std::vector<Path>& all_paths, std::vector<bool> to_delete) const
{
    unsigned int dst = 0;
    for (unsigned int src = 0; src < all_paths.size(); ++src)
    {
        Path& path = all_paths[src];
        unsigned int i = 0;
        for (i = 0; i < path.size(); ++i)
        {
            if (to_delete[path[i].first])
                break;
        }
        if (i == path.size())
        {
            if (src != dst)
                all_paths[dst] = all_paths[src];
            ++dst;
        }
    }
    all_paths.resize(dst);
}

void Plan::filterDeadEndPaths(std::vector<Path>& all_paths) const
{
    int target_idx = findEndCorridor();
    unsigned int dst = 0;
    for (unsigned int src = 0; src < all_paths.size(); ++src)
    {
        if (all_paths[src].back().first == target_idx)
        {
            if (src != dst)
                all_paths[dst] = all_paths[src];
            ++dst;
        }
    }
    all_paths.resize(dst);
}


bool Plan::removeDeadEnds()
{
    DEBUG_OUT("removing dead ends");
    int start_corridor = findStartCorridor();
    int end_corridor   = findEndCorridor();
    set<int> end_corridors;
    end_corridors.insert(start_corridor);
    end_corridors.insert(end_corridor);
    return removeDeadEnds(end_corridors);
}

bool Plan::removeDeadEnds(set<int> keepalive)
{
    // Each tuple is
    //  has_front_in_connection[0], has_front_out_connection[1], has_back_in_connection[2], has_back_out_connection[3], has_multiple_neighbours[4], last_neighbour[5]
    typedef tuple<bool, bool, bool, bool, bool, int> DeadEndState;
    vector<DeadEndState> states;
    states.resize(corridors.size(), DeadEndState(false, false, false, false, false, -1));

    for (int corridor_idx = 0; corridor_idx < (int)corridors.size(); ++corridor_idx)
    {
        DEBUG_OUT("corridor " << corridors[corridor_idx].name << ": ");
        DeadEndState& this_state = states[corridor_idx];

        Corridor::Connections const& connections = corridors[corridor_idx].connections;
        for (Corridor::const_connection_iterator conn_it = connections.begin();
                conn_it != connections.end(); ++conn_it)
        {
            DEBUG_OUT("   conn: " << conn_it->this_side << " " << corridors[conn_it->target_idx].name << " " << conn_it->target_side);
            this_state.get<1>() |= !conn_it->this_side;
            this_state.get<3>() |=  conn_it->this_side;
            this_state.get<4>() |= (this_state.get<5>() != -1 && this_state.get<5>() != conn_it->target_idx);
            this_state.get<5>() = conn_it->target_idx;
            DEBUG_OUT("   state[" << corridors[corridor_idx].name << "]: " << this_state.get<0>() << " " << this_state.get<1>() << " " 
                    << this_state.get<2>() << " " << this_state.get<3>() << " "
                    << this_state.get<4>() << " " << corridors[this_state.get<5>()].name);

            DeadEndState& target_state = states[conn_it->target_idx];
            target_state.get<0>() |= !conn_it->target_side;
            target_state.get<2>() |=  conn_it->target_side;
            target_state.get<4>() |= (target_state.get<5>() != -1 && target_state.get<5>() != corridor_idx);
            target_state.get<5>() = corridor_idx;
            DEBUG_OUT("   state[" << corridors[conn_it->target_idx].name << "]: " << target_state.get<0>() << " " << target_state.get<1>() << " " 
                    << target_state.get<2>() << " " << target_state.get<3>() << " "
                    << target_state.get<4>() << " " << corridors[target_state.get<5>()].name);
        }
    }

    for (int idx = states.size() - 1; idx >= 0; --idx)
    {
        if (keepalive.count(idx))
            continue;

        DeadEndState state = states[idx];
        bool is_traversed = (state.get<0>() && state.get<3>()) || (state.get<1>() && state.get<2>());
        // bool is_traversed = (state.get<0>() || state.get<1>()) || (state.get<2>() && state.get<3>());
        if (!(is_traversed && state.get<4>()))
        {
            DEBUG_OUT("corridor " << corridors[idx].name << " is a dead end");
            removeCorridor(idx);
        }
    }
    if (corridors.size() != states.size()) // we have removed some corridors
    {
        removeDeadEnds();
        return true;
    }
    return false;
}

void Plan::removeNullCorridors()
{
    int start_corridor = findStartCorridor();
    int end_corridor   = findEndCorridor();
    set<int> end_corridors;
    end_corridors.insert(start_corridor);
    end_corridors.insert(end_corridor);
    removeNullCorridors(end_corridors);
}

bool Plan::filterNullSingleton(int corridor_idx)
{
    Corridor& corridor = corridors[corridor_idx];
    set<int> connected_to = corridor.connectivity();
    if (connected_to.size() < 2)
        return true;

    while (!connected_to.empty())
    {
        int source_idx = *(connected_to.begin());
        connected_to.erase(connected_to.begin());

        set<int>::const_iterator missing = find_if(connected_to.begin(), connected_to.end(),
                !bind(&Corridor::isConnectedTo, ref(corridors[source_idx]), _1));

        if (missing == connected_to.end())
            corridor.removeConnectionsTo(source_idx);
    }
    return (corridor.connections.size() < 2);
}

void Plan::removeNullCorridors(set<int> keepalive)
{
    set<int> to_remove;

    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        if (keepalive.count(corridor_idx))
            continue;

        if (corridors[corridor_idx].isSingleton() && filterNullSingleton(corridor_idx))
                to_remove.insert(corridor_idx);
    }

    if (!to_remove.empty())
    {
#ifdef DEBUG
        cerr << "found the following null corridors: ";
        copy(to_remove.begin(), to_remove.end(),
                ostream_iterator<int>(cerr, ", "));
        cerr << endl;
#endif

        removeCorridors(to_remove);
        removeNullCorridors();
    }
}

bool Plan::markDirections_DFS(std::set< tuple<int, bool, int, bool> >& result, 
	std::vector<int>& stack, int in_side, int idx, int end_idx,
	float accumulated_cost_overhead, float cost_margin)
{
    static std::string indent; // for debugging purposes only

    if (end_idx == idx)
    {
        DEBUG_OUT(indent << "reached the end");
	return true;
    }

    Corridor& corridor = corridors[idx];
    stack.push_back(idx);

    indent += "  ";

    // Some values about our input and output points
    int in_type  = corridor.end_types[in_side];
    int out_side = !in_side;
    float dcost = corridor.getCostDelta(in_side);

    DEBUG_OUT(indent << "looking at " << corridor.name << ", cost_overhead=" << accumulated_cost_overhead << ", margin=" << cost_margin << ", dcost=" << dcost);

    // Check the currently known type for this side. If we are going backwards,
    // we'll have to check the cost threshold
    bool backwards = false;
    if (in_type == ENDPOINT_BACK)
    {
	DEBUG_OUT(indent << "taking it backwards");
        if (dcost < 0)
            throw logic_error(corridor.name + " is supposed to be taken backwards, but dCost=" + lexical_cast<string>(dcost));

	backwards = true;
	accumulated_cost_overhead += dcost;
	if (accumulated_cost_overhead > cost_margin)
	{
	    DEBUG_OUT(indent << "reached cost margin, cannot traverse this corridor");
	    indent.resize(indent.size() - 2);
	    DEBUG_OUT(indent << "done " << corridors[idx].name);
	    stack.pop_back();
	    return false;
	}
    }

    if (reach_flag[2 * idx + in_side])
    {
	DEBUG_OUT(indent << "corridor " << corridors[idx].name << " reaches the exit with cost " << reach_min_cost[2 * idx + in_side]);
	indent.resize(indent.size() - 2);
	DEBUG_OUT(indent << "done " << corridors[idx].name);
	stack.pop_back();
	return (accumulated_cost_overhead + reach_min_cost[2 * idx + in_side] < cost_margin);
    }

    Corridor::Connections& connections = corridor.connections;
    Corridor::connection_iterator it = connections.begin(),
        end = connections.end();

    set<int> seen;
    for (it = connections.begin(); it != end; ++it)
    {
	// We look only at points "at the other side" of the corridor.
	if (it->this_side != out_side)
	    continue;

        int target_idx = it->target_idx;

	// And we forget about loops
        if (find(stack.begin(), stack.end(), target_idx) != stack.end())
	{
	    DEBUG_OUT(indent << corridors[target_idx].name << " is on the stack");
            continue;
	}

	Corridor& target_corridor = corridors[target_idx];
	int target_side = it->target_side;

	if (markDirections_DFS(result, stack, target_side, target_idx, end_idx, accumulated_cost_overhead, cost_margin))
	{
	    DEBUG_OUT(indent << "reached end point through " << corridors[target_idx].name);
	    DEBUG_OUT(indent << "keeping connection");
	    result.insert( make_tuple(idx, it->this_side, target_idx, it->target_side) );

            int& orientation = orientations[2 * idx + out_side];
            if (ENDPOINT_UNKNOWN == orientation)
                orientation = ENDPOINT_BACK;
            else if (ENDPOINT_FRONT == orientation)
            {
                DEBUG_OUT(indent << corridor.name << " is bidirectional");
                corridor.bidirectional = true;
                orientations[2 * idx + in_side] = ENDPOINT_BIDIR;
                orientation = ENDPOINT_BIDIR;
            }

            int& target_orientation = orientations[2 * target_idx + target_side];
            if (ENDPOINT_UNKNOWN == target_orientation)
                target_orientation = ENDPOINT_FRONT;
            else if (ENDPOINT_BACK == target_orientation)
            {
                DEBUG_OUT(indent << target_corridor.name << " is bidirectional");
                target_corridor.bidirectional = true;
                target_orientation = ENDPOINT_BIDIR;
                orientations[2 * target_idx + !target_side] = ENDPOINT_BIDIR;
            }

	    float min_cost = reach_min_cost[2 * target_idx + target_side];
	    if (backwards)
		min_cost += dcost;
	    if (reach_flag[2 * idx + in_side])
		min_cost = min(reach_min_cost[2 * idx + in_side], min_cost);
	    DEBUG_OUT(indent << "min cost is now " << min_cost);

	    reach_flag[2 * idx + in_side] = true;
	    reach_min_cost[2 * idx + in_side] = min_cost;
	}
    }

    stack.pop_back();

    indent.resize(indent.size() - 2);
    DEBUG_OUT(indent << "done " << corridors[idx].name);
    return reach_flag[2 * idx + in_side];
}

void Plan::markDirections_cost()
{
    // Now, try to determine the leftovers (i.e. the bidirectional corridors) by
    // looking at the cost map. Namely, we look at the maximal cost difference
    // between each connection targets. Then, the upper half is back and the
    // bottom half is front (remember that we are going down the cost).
    //
    // There is no need to look at corridors which are of unknown cost: if they
    // have not been seen by the DFS it means they are useless.
    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];

        // Gather a cost value for each end regions (mean value)
        Corridor::Connections::const_iterator conn_it;

	float costs[2]  = { 0, 0 };
        int   counts[2] = { 0, 0 };
        for (conn_it = corridor.connections.begin(); conn_it != corridor.connections.end(); ++conn_it)
        {
            PointID p = corridors[conn_it->target_idx].getEndpoint(conn_it->target_side);
            costs[conn_it->this_side] += m_nav_function.getValue(p.x, p.y);
            counts[conn_it->this_side]++;
        }

        if (counts[0] == 0 || counts[1] == 0)
        {
	    DEBUG_OUT("  " << corridor.name << " is a dead end");
            continue;
        }

        corridor.dcost = costs[1] / counts[1] - costs[0] / counts[0];
        if (corridor.dcost > 0)
            reverseCorridor(corridor_idx);

        corridor.end_types[0] = ENDPOINT_FRONT;
        corridor.end_types[1] = ENDPOINT_BACK;

        DEBUG_OUT("corridor " << corridor.name << " is oriented FRONT => BACK as " << corridor.frontPoint() << " => " << corridor.backPoint() << " dcost=" << corridor.dcost);
    }
}

void Plan::removeBackToBackConnections(double margin_factor)
{
    size_t start_idx = findStartCorridor();
    size_t end_idx   = findEndCorridor();

    markDirections_cost();

    Corridor const& start_corridor = corridors[start_idx];
    float cost_margin = m_nav_function.getValue(m_start.x, m_start.y) * (margin_factor - 1.0);

    reach_flag.resize(2 * corridors.size());
    fill(reach_flag.begin(), reach_flag.end(), false);
    reach_min_cost.resize(2 * corridors.size());
    orientations.resize(2 * corridors.size());
    fill(orientations.begin(), orientations.end(), 0);

    reach_flag[2 * end_idx] = true;
    reach_flag[2 * end_idx + 1] = true;
    reach_min_cost[2 * end_idx] = 0;
    reach_min_cost[2 * end_idx + 1] = 0;

    std::set< boost::tuple<int, bool, int, bool> > result;
    for (Corridor::Connections::const_iterator it = start_corridor.connections.begin(); it != start_corridor.connections.end(); ++it)
    {
	int  target_idx  = it->target_idx;
	bool target_side = it->target_side;
	DEBUG_OUT("initializing DFS-based DAG convertion with " << corridors[target_idx].name << " (side=" << target_side << ")");
        DEBUG_OUT("   start corridor:" << corridors[start_idx].name);
        DEBUG_OUT("   end corridor:" << corridors[end_idx].name);

	vector<int> stack;
	if (markDirections_DFS(result, stack, target_side, target_idx, end_idx, 0, cost_margin))
        {
            orientations[target_idx * 2 + target_side] = ENDPOINT_FRONT;
            result.insert( make_tuple(start_idx, it->this_side, target_idx, target_side) );
        }
    }

    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
	Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator conn_it = connections.begin(),
            end = connections.end();
        while (conn_it != end)
	{
	    if (result.find( make_tuple(corridor_idx, conn_it->this_side, conn_it->target_idx, conn_it->target_side) ) == result.end())
		connections.erase(conn_it++);
	    else ++conn_it;
	}
    }
}

int Plan::findStartCorridor() const
{
    for (size_t owner = 0; owner < corridors.size(); ++owner)
        if (corridors[owner].isSingleton() && corridors[owner].front().center == m_start)
            return owner;

    throw runtime_error("no corridor for start point");
}
int Plan::findEndCorridor() const
{
    for (size_t owner = 0; owner < corridors.size(); ++owner)
        if (corridors[owner].isSingleton() && corridors[owner].front().center == m_end)
            return owner;

    throw runtime_error("no corridor for start point");
}
int Plan::findCorridorOf(PointID const& endp) const
{
    float min_distance = -1;
    int owner = -1;

    for (int i = corridors.size() - 1; i >= 0; --i)
    {
        Corridor const& corridor = corridors[i];
        if (!corridor.bbox.isNeighbour(endp))
        {
            float d = min(corridor.frontPoint().distance2(endp), corridor.backPoint().distance2(endp));
            if (min_distance > 0 && d > min_distance)
                continue;
        }

        Corridor::voronoi_const_iterator nearest = corridor.findNearestMedian(endp);
        float d = endp.distance2(nearest->center);
        if (min_distance < 0 || min_distance > d)
        {
            min_distance = d;
            owner = i;
            if (min_distance == 0)
                break;
        }
    }

    if (owner == -1)
        throw std::runtime_error("no owner for the given point");

    DEBUG_OUT("corridor " << corridors[owner].name << " is closest to " << endp << " with d == " << min_distance);
    return owner;
}

static int solveMergeMappings(map<int, int> const& merge_mappings, int idx)
{
    map<int, int>::const_iterator it = merge_mappings.find(idx);
    while (it != merge_mappings.end())
    {
        idx = it->second;
        it = merge_mappings.find(idx);
    }
    return idx;
}

void Plan::mergeSimpleCrossroads_directed()
{
    DEBUG_OUT("removing simple crossroads");
    int start_corridor = findStartCorridor();
    int end_corridor   = findEndCorridor();

    map< Endpoint, int > endpoint_to_crossroad;
    vector< list<Endpoint> > crossroads;

    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor::Connections& connections = corridors[corridor_idx].connections;

        bool not_already_there;
        for (int side = 0; side < 2; ++side)
        {
            Endpoint this_endpoint(corridor_idx, side);

            // Check if there is already an owner for this endpoint, and add it
            // if it is not the case.
            map<Endpoint, int>::iterator endpoint_owner;
            tie(endpoint_owner, not_already_there) =
                endpoint_to_crossroad.insert( make_pair(this_endpoint, crossroads.size()) );
            if (not_already_there)
            {
                crossroads.push_back( list<Endpoint>() );
                crossroads.back().push_back(this_endpoint);
            }

            int crossroad_idx = endpoint_owner->second;
            list<Endpoint>& crossroad = crossroads[crossroad_idx];

            // Check all the connections. Either merge adjacent crossroads, or
            // add the adjacent points to +crossroad+.
            Corridor::const_connection_iterator conn_it;
            for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
            {
                if (conn_it->this_side != side) continue;

                Endpoint target_endpoint(conn_it->target_idx, conn_it->target_side);
                map<Endpoint, int>::iterator target_owner;
                tie(target_owner, not_already_there) =
                    endpoint_to_crossroad.insert( make_pair(target_endpoint, crossroad_idx) );
                if (not_already_there)
                    crossroad.push_back(target_endpoint);
                else if (target_owner->second != crossroad_idx) 
                {
                    list<Endpoint>& old_crossroad = crossroads[target_owner->second];
                    for (list<Endpoint>::const_iterator endp_it = old_crossroad.begin();
                            endp_it != old_crossroad.end(); ++endp_it)
                        endpoint_to_crossroad[*endp_it] = crossroad_idx;
                    crossroad.splice(crossroad.end(), old_crossroad);
                }
            }
        }
    }

    set<int> to_remove;
    for (int corridor_idx = 0; corridor_idx < (int)corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        if (!corridor.isSingleton()) continue;

        int in_idx = endpoint_to_crossroad[ Endpoint(corridor_idx, false) ];
        list<Endpoint>& in_crossroad = crossroads[in_idx];
        int out_idx = endpoint_to_crossroad[ Endpoint(corridor_idx, true) ];
        list<Endpoint>& out_crossroad = crossroads[out_idx];

        if (in_crossroad.empty() || out_crossroad.empty())
        {
            DEBUG_OUT(corridor.name << " is a null corridor");
            to_remove.insert(corridor_idx);
        }
    }

    // Mapping from an already merged corridor to the corridor into which is has
    // been merged
    DEBUG_OUT("found " << crossroads.size() << " crossroads");
    map<int, int> merge_mappings;
    for (int crossroad_idx = 0; crossroad_idx < (int)crossroads.size(); ++crossroad_idx)
    {
        DEBUG_OUT("crossroad " << crossroad_idx << " has " << crossroads[crossroad_idx].size() << " endpoints");
        if (crossroads[crossroad_idx].size() != 2)
            continue;

        Endpoint c0_endp = crossroads[crossroad_idx].front();
        int c0_idx = c0_endp.corridor_idx;
        c0_idx = solveMergeMappings(merge_mappings, c0_idx);
        Corridor& c0 = corridors[c0_idx];

        Endpoint c1_endp = crossroads[crossroad_idx].back();
        int c1_idx = c1_endp.corridor_idx;
        c1_idx = solveMergeMappings(merge_mappings, c1_idx);
        Corridor& c1 = corridors[c1_idx];

        if (c0.bidirectional ^ c1.bidirectional)
            continue;
        if (c0_idx == start_corridor || c1_idx == start_corridor ||
                c0_idx == end_corridor || c1_idx == end_corridor)
            continue;

        if (c1_endp.side == c0_endp.side)
            reverseCorridor(c1_idx);


        if (c0_endp.side == false)
        {
            DEBUG_OUT("concatenating " << c0.name << " at the end of " << c1.name);
            // grafting c1.back onto
            moveConnections(c0_idx, c1_idx);
            c1.concat(c0);
            merge_mappings[c0_idx] = c1_idx;
            to_remove.insert(c0_idx);
        }
        else
        {
            DEBUG_OUT("concatenating " << c1.name << " at the end of " << c0.name);
            moveConnections(c1_idx, c0_idx);
            c0.concat(c1);
            merge_mappings[c1_idx] = c0_idx;
            to_remove.insert(c1_idx);
        }
    }

    for (set<int>::const_reverse_iterator it = to_remove.rbegin();
            it != to_remove.rend(); ++it)
        removeCorridor(*it);
}

void Plan::checkConsistency() const
{
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor const& corridor = corridors[i];
        corridor.checkConsistency();

	//size_t end_regions = corridor.endRegions().size();
	//if (!corridor.isSingleton() && end_regions != 2)
	//{
	//    for (Corridor::voronoi_const_iterator it = corridor.median.begin();
	//	    it != corridor.median.end(); ++it)
	//	DEBUG_OUT(" " << it->center);

	//    DEBUG_OUT(end_regions << " end regions for corridor " << corridor.name);
	//}

        Corridor::Connections const& connections = corridor.connections;
        for (Corridor::Connections::const_iterator it = connections.begin();
                it != connections.end(); ++it)
        {
            size_t target_idx = it->target_idx;
            if (target_idx == i)
                DEBUG_OUT("  " << corridor.name << " is looping on itself");

            if (target_idx < 0 || target_idx >= corridors.size())
                DEBUG_OUT("  wrong target index in connection from " << corridor.name << ": " << target_idx);
        }
    }
}

ostream& corridor_planner::operator << (ostream& io, Plan const& plan)
{
    int const NO_OWNER = plan.corridors.size();
    int const width    = plan.width;
    int const height   = plan.height;

    if (!plan.pixel_map.empty())
    {
        io << "\nPixel map\n";
        io << "  ";
        for (int x = 0; x < width; ++x)
            io << " " << std::setw(2) << x;
        io << endl;
        for (int y = 0; y < height; ++y)
        {
            io << std::setw(2) << y;
            for (int x = 0; x < width; ++x)
            {
                int owner = plan.pixel_map[ x + y * width ];
                if (owner != NO_OWNER)
                    io << " " << std::setw(2) << owner;
                else
                    io << "  -";
            }
            io << endl;
        }
    }

    for (vector<Corridor>::const_iterator it = plan.corridors.begin(); it != plan.corridors.end(); ++it)
    {
        io << "\n==== Corridor " << it - plan.corridors.begin() << "====\n";
        io << *it << endl;
    }
    return io;
}

void Plan::reverseCorridor(int corridor_idx)
{
    Corridor& corridor = corridors[corridor_idx];
    corridor.reverse();

    for (Corridor::Connections::iterator it = corridor.connections.begin();
            it != corridor.connections.end(); ++it)
    {
        it->this_side = !it->this_side;
    }

    for (int i = 0; i < (int)corridors.size(); ++i)
    {
        if (i == corridor_idx) continue;
        Corridor& corridor = corridors[i];
        for (Corridor::Connections::iterator it = corridor.connections.begin();
                it != corridor.connections.end(); ++it)
        {
            if (it->target_idx == corridor_idx)
                it->target_side = !it->target_side;
        }
    }
}

bool Plan::removeUselessCorridorConnections()
{
    bool did_something = true, result = false;
    while (did_something)
    {
        did_something = false;
        for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
        {
            if (removeUselessCorridorConnections(corridor_idx))
                did_something = result = true;
        }
    }

    return result;
}

bool Plan::removeUselessCorridorConnections(int corridor_idx)
{
    Corridor& corridor = corridors[corridor_idx];

    set<int> connectivity[2];
    set< pair<bool, int> > useless;
    for (Corridor::const_connection_iterator conn_it = corridor.connections.begin();
            conn_it != corridor.connections.end(); ++conn_it)
    {
        bool already_there = !connectivity[conn_it->this_side].
            insert(conn_it->target_idx).second;
        if (already_there)
        {
            DEBUG_OUT("corridor " << corridor.name << ": useless connection from " << conn_it->this_side
                    << " to " << corridors[conn_it->target_idx].name);
            useless.insert( make_pair(conn_it->this_side, conn_it->target_idx) );
        }
    }

    if (useless.empty())
        return false;

    for (set< pair<bool, int> >::iterator it = useless.begin();
            it != useless.end(); ++it)
    {
        bool this_side  = it->first;
        int useless_idx = it->second;
        Corridor& useless_corridor = corridors[useless_idx];
        for (Corridor::const_connection_iterator conn_it = useless_corridor.connections.begin();
                conn_it != useless_corridor.connections.end(); ++conn_it)
        {
            if (conn_it->target_idx != corridor_idx)
                corridor.addConnection(this_side, conn_it->target_idx, conn_it->target_side);
        }
        corridor.removeConnectionsTo(useless_idx);
    }
    return true;
}

void Plan::displayConnections(bool include_corridor_inner_connections) const
{
    for (unsigned int i = 0; i < corridors.size(); ++i)
    {
        if (include_corridor_inner_connections)
        {
            std::cout << "FRONT_" << corridors[i].name << " -> " << "BACK_" << corridors[i].name << std::endl;
            std::cout << "BACK_" << corridors[i].name << " -> " << "FRONT_" << corridors[i].name << std::endl;
        }

        Corridor const& corridor = corridors[i];
        Corridor::Connections const& connections = corridor.connections;
        for (Corridor::Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
            std::cout << (it->this_side ? "BACK" : "FRONT") << "_" << corridors[i].name << " -> " << (it->target_side ? "BACK" : "FRONT") << "_" << corridors[it->target_idx].name << std::endl;
    }
}

