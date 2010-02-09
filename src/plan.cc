#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>
#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple_comparison.hpp>

// #include <CGAL/Cartesian.h>
// #include <CGAL/convex_hull_2.h>

const int nav::Plan::USEFUL;
const int nav::Plan::NOT_USEFUL;

using namespace std;
using namespace nav;
using namespace boost;

Plan::Plan() {}
Plan::Plan(PointID start, PointID end, GridGraph const& nav_function)
    : m_start(start), m_end(end), m_nav_function(nav_function) {}

PointID Plan::getStartPoint() const { return m_start; }
PointID Plan::getEndPoint() const { return m_end; }
GridGraph const& Plan::getNavigationFunction() const { return m_nav_function; }

void Plan::setStartPoint(PointID const& p) { m_start = p; }
void Plan::setEndPoint(PointID const& p) { m_end = p; }
void Plan::setNavigationFunction(GridGraph const& nav_function)
{ m_nav_function = nav_function; }

void Plan::clear()
{ corridors.clear(); }

void Plan::removeCorridor(int idx)
{
    corridors.erase(corridors.begin() + idx);
    for (corridor_iterator corridor = corridors.begin(); corridor != corridors.end(); ++corridor)
    {
        Corridor::Connections& connections = corridor->connections;
        Corridor::connection_iterator it = connections.begin();
        while (it != connections.end())
        {
            int const target_idx = it->get<1>();
            if (target_idx == idx)
                connections.erase(it++);
            else
            {
                if (target_idx > idx)
                    it->get<1>()--;

                ++it;
            }
        }
    }
}

void Plan::concat(Plan const& other)
{
    int merge_start = corridors.size();
    copy(other.corridors.begin(), other.corridors.end(), back_inserter(corridors));

    for (vector<Corridor>::iterator it = corridors.begin() + merge_start; it != corridors.end(); ++it)
        for (Corridor::connection_iterator c = it->connections.begin(); c != it->connections.end(); ++c)
            c->get<1>() += merge_start;
}

void Plan::moveConnections(size_t into_idx, size_t from_idx)
{
    Corridor::Connections& from = corridors[from_idx].connections;
    Corridor::Connections& into = corridors[into_idx].connections;
    Corridor::Connections::iterator conn_it;

    conn_it = into.begin();
    while (conn_it != into.end())
    {
        size_t target_idx = conn_it->get<1>();
        if (target_idx == from_idx)
            into.erase(conn_it++);
        else ++conn_it;
    }

    conn_it = from.begin();
    while (conn_it != from.end())
    {
        size_t target_idx = conn_it->get<1>();
        if (target_idx != into_idx)
            into.splice(into.end(), from, conn_it++);
        else
            ++conn_it;
    }

    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (i == into_idx || i == from_idx)
            continue;

        Corridor::Connections& connections = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            if (conn_it->get<1>() == (int)from_idx)
                conn_it->get<1>() = into_idx;
        }
    }
}

pair<PointID, PointID> Plan::split(int corridor_idx, Corridor::voronoi_iterator it)
{
    Corridor& corr = corridors[corridor_idx];

    Corridor new_corr;
    // TODO: better bounding boxes
    new_corr.bbox         = corr.bbox;
    new_corr.median_bbox = corr.median_bbox;

    // We know the orientation of the median line is BACK => FRONT. Therefore,
    // we just have to splice it
    new_corr.voronoi.splice(new_corr.end(), corr.voronoi, it, corr.voronoi.end());

    // Check that both result corridors are not empty
    if (new_corr.voronoi.empty())
        throw std::runtime_error("split() leads to an empty left corridor");
    if (corr.voronoi.empty())
        throw std::runtime_error("split() leads to an empty right corridor");

    // Take one element of a border in +it+, find in which corridor border it
    // is, and split the boundary at that point
    //
    // Repeat for the other border.
    VoronoiPoint const& last_corr_point      = corr.voronoi.back();
    {
        VoronoiPoint::BorderList::const_iterator it = last_corr_point.borders.begin();
        VoronoiPoint::BorderList::const_iterator const end = last_corr_point.borders.end();

        for (; it != end; ++it)
        {
            if (it->empty())
                throw std::logic_error("empty border found");

            PointID p = it->front();
            list<PointID>::iterator p_it =
                find(corr.boundaries[0].begin(), corr.boundaries[0].end(), p);
            if (p_it != corr.boundaries[0].end())
            {
                new_corr.boundaries[0].splice(
                        new_corr.boundaries[0].end(), corr.boundaries[0],
                        p_it, corr.boundaries[0].end());
            }
            else
            {
                p_it = find(corr.boundaries[1].begin(), corr.boundaries[1].end(), p);
                if (p_it != corr.boundaries[1].end())
                {
                    new_corr.boundaries[1].splice(
                            new_corr.boundaries[1].end(), corr.boundaries[1],
                            p_it, corr.boundaries[1].end());
                }
                else
                {
                    throw std::logic_error("internal error: cannot find a boundary point correspondance");
                }
            }
        }
    }

    // Generate the border sets and gather all center points in median_point_set
    // to update the connections later on
    map<PointID, int> ownerships;
    for (Corridor::voronoi_iterator median_it = new_corr.begin(); median_it != new_corr.end(); ++median_it)
        ownerships[median_it->center] = 1;

    for (Corridor::voronoi_iterator median_it = corr.begin(); median_it != corr.end(); ++median_it)
        ownerships[median_it->center] |= 2;

    // Update the connection that go to +corridor_idx+
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (i == (size_t)corridor_idx)
            continue;

        Corridor::Connections& connections = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            if (conn_it->get<1>() == corridor_idx)
            {
                int mask = ownerships[conn_it->get<2>()];
                if (mask == 1) // only owned by new_corr
                    conn_it->get<1>() = corridors.size();
                else if (mask == 3) // owned by both sides
                    corridors[i].addConnection(conn_it->get<0>(), corridors.size(), conn_it->get<2>());
            }
        }
    }

    // Update the connections which come from +corridor_idx+
    Corridor::Connections& connections = corr.connections;
    Corridor::Connections::iterator conn_it = connections.begin();
    while (conn_it != connections.end())
    {
        int mask = ownerships[conn_it->get<0>()];
        new_corr.addConnection(conn_it->get<0>(), conn_it->get<1>(), conn_it->get<2>());
        if (mask == 1)
            connections.erase(conn_it++);
        else ++conn_it;
    }

    corridors.push_back(new_corr);
    corridors.back().name = corr.name + "/2";
    corr.name += "/1";
    return make_pair(corr.backPoint(), corridors.back().frontPoint());
}

void Plan::createEndpointCorridor(PointID const& endpoint, int direction, std::string const& name)
{
    Corridor  endp_corridor = Corridor::singleton(endpoint, name);
    endp_corridor.end_types[0] = direction;

    int current_idx = findCorridorOf(endpoint);
    Corridor& current_corridor = corridors[current_idx];

    Corridor::voronoi_iterator it = current_corridor.findNearestMedian(endpoint);

    if (it == current_corridor.begin() || it == (--current_corridor.end()))
    {
        // The end point does not split a corridor in two, just connect
        int endp_idx = corridors.size();
        PointID conn_p = it->center;
        endp_corridor.addConnection(endpoint, current_idx, conn_p);
        current_corridor.addConnection(conn_p, endp_idx, endpoint);

        corridors.push_back(endp_corridor);
    }
    else
    {
        // We need to split +current_corridor+ in two
        cerr << "splitting " << current_corridor.name << flush;
        current_corridor.checkConsistency();

        pair<PointID, PointID> new_endpoints = split(current_idx, it);
        int left_idx  = current_idx;
        Corridor& left_corridor = corridors[left_idx];
        int right_idx = corridors.size() - 1;
        Corridor& right_corridor = corridors[right_idx];
        std::cerr << ", created " << right_corridor.name << endl;

        current_corridor.checkConsistency();
        right_corridor.checkConsistency();

        // WARN: endp_idx must be initialized here, as split() adds a new
        // WARN: corridor to the corridor set, and therefore changes the index
        // WARN: of the endpoint corridor !
        int endp_idx = corridors.size();
        endp_corridor.addConnection(endpoint, left_idx, new_endpoints.first);
        left_corridor.addConnection(new_endpoints.first, endp_idx, endpoint);
        endp_corridor.addConnection(endpoint, right_idx, new_endpoints.second);
        right_corridor.addConnection(new_endpoints.second, endp_idx, endpoint);

        corridors.push_back(endp_corridor);

        if (left_corridor.isSingleton())
        {
            moveConnections(right_idx, left_idx);
            if (right_corridor.isSingleton())
                throw std::logic_error("both corridors are singletons");
            removeCorridor(left_idx);
        }
        else if (right_corridor.isSingleton())
        {
            moveConnections(left_idx, right_idx);
            removeCorridor(right_idx);
        }
    }
}

void Plan::simplify()
{
    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);

    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& c = corridors[corridor_idx];
        c.fixLineOrderings();
    }
    cerr << "after fixing line orderings" << endl;
    checkConsistency();

    // BIG FAT NOTE: cannot use findStartCorridor() here as the start corridor
    // is not yet created.
    useful_corridors[findCorridorOf(m_start)] = USEFUL;
    useful_corridors[findCorridorOf(m_end)]   = USEFUL;
    markNullCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    markUselessCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    cerr << "after simplification pass on undirected graph" << endl;
    checkConsistency();

    // Create two singleton corridors for the start and end points
    createEndpointCorridor(m_start, ENDPOINT_FRONT, lexical_cast<string>(corridors.size()) + "_start");
    createEndpointCorridor(m_end,   ENDPOINT_BACK,  lexical_cast<string>(corridors.size()) + "_end");
    cerr << "created start and end corridors" << endl;

    cerr << "end regions:" << endl;
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor& corridor = corridors[i];
        corridor.buildEndRegions();

        cerr << "  " << corridors[i].name << ":" << endl;
        if (corridor.end_regions[0].empty() || corridor.end_regions[1].empty())
        {
            int line_count = 0;
            for (Corridor::voronoi_const_iterator median_it = corridor.voronoi.begin();
                    median_it != corridor.voronoi.end(); ++median_it)
            {
                if (++line_count > 10)
                {
                    cerr << endl << "    ";
                    line_count = 0;
                }
                cerr << " " << median_it->center;
            }
            cerr << endl;
        }

        for (int j = 0; j < 2; ++j)
            cerr << "    " << corridors[i].end_regions[j] << endl;
    }

    removeBackToBackConnections();

    mergeSimpleCrossroads_directed();

    cerr << "merged simple crossroads" << endl;
    checkConsistency();
}

bool Plan::markDirections_DFS(std::set< tuple<int, PointID, int, PointID> >& result, 
	std::vector<int>& stack, int in_side, int idx, int end_idx,
	float accumulated_cost_overhead, float cost_margin)
{
    static std::string indent; // for debugging purposes only

    if (end_idx == idx)
	return true;

    Corridor& corridor = corridors[idx];
    if (corridor.isSingleton())
	return false;

    if (corridor.end_regions[0].empty() || corridor.end_regions[1].empty())
    {
	cerr << "only one end region in corridor " << corridor.name << endl;
	return false; // only one endpoint ? No luck ...
    }

    stack.push_back(idx);

    cerr << indent << "looking at " << corridor.name << ", cost_overhead=" << accumulated_cost_overhead << ", margin=" << cost_margin << endl;
    indent += "  ";

    // Some values about our input and output points
    int     in_type  = corridor.end_types[in_side];
    PointID in_p     = *corridor.end_regions[in_side].begin();
    float   in_cost  = m_nav_function.getValue(in_p.x, in_p.y);
    int     out_side = !in_side;
    PointID out_p    = *corridor.end_regions[out_side].begin();
    float   out_cost = m_nav_function.getValue(out_p.x, out_p.y);

    // Check the currently known type for this side. If we are going backwards,
    // we'll have to check the cost threshold
    bool backwards = false;
    if (in_type == ENDPOINT_FRONT)
    {
	cerr << indent << "taking it backwards" << endl;
        if (out_cost < in_cost)
            throw logic_error(corridor.name + " is supposed to be taken backwards, but dCost=" + lexical_cast<string>(out_cost - in_cost));

	backwards = true;
	accumulated_cost_overhead += (out_cost - in_cost);
	if (accumulated_cost_overhead > cost_margin)
	{
	    cerr << indent << "reached cost margin, cannot traverse this corridor" << endl;
	    indent.resize(indent.size() - 2);
	    cerr << indent << "done " << corridors[idx].name << endl;
	    stack.pop_back();
	    return false;
	}
    }

    if (reach_flag[2 * idx + in_side])
    {
	cerr << indent << "corridor " << corridors[idx].name << " reaches the exit with cost " << reach_min_cost[2 * idx + in_side] << endl;
	indent.resize(indent.size() - 2);
	cerr << indent << "done " << corridors[idx].name << endl;
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
	if (!corridor.end_regions[out_side].count(it->get<0>()))
	    continue;

        int target_idx = it->get<1>();

	// And we forget about loops
        if (find(stack.begin(), stack.end(), target_idx) != stack.end())
	{
	    cerr << indent << corridors[target_idx].name << " is on the stack" << endl;
            continue;
	}

	Corridor& target_corridor = corridors[target_idx];
	int target_side = target_corridor.findSideOf(it->get<2>());

	if (markDirections_DFS(result, stack, target_side, target_idx, end_idx, accumulated_cost_overhead, cost_margin))
	{
	    cerr << indent << "reached end point through " << corridors[target_idx].name << endl;
	    cerr << indent << "keeping connection to " << it->get<2>() << endl;
	    result.insert( make_tuple(idx, it->get<0>(), target_idx, it->get<2>()) );

            int& orientation = orientations[2 * idx + out_side];
            if (ENDPOINT_UNKNOWN == orientation)
                orientation = ENDPOINT_FRONT;
            else if (ENDPOINT_BACK == orientation)
            {
                corridor.bidirectional = true;
                orientations[2 * idx + in_side] = ENDPOINT_BIDIR;
                orientation = ENDPOINT_BIDIR;
            }

            int& target_orientation = orientations[2 * target_idx + target_side];
            if (ENDPOINT_UNKNOWN == target_orientation)
                target_orientation = ENDPOINT_BACK;
            else if (ENDPOINT_FRONT == target_orientation)
            {
                target_corridor.bidirectional = true;
                target_orientation = ENDPOINT_BIDIR;
                orientations[2 * target_idx + !target_side] = ENDPOINT_BIDIR;
            }

	    float min_cost = reach_min_cost[2 * target_idx + target_side];
	    if (backwards)
		min_cost += (out_cost - in_cost);
	    if (reach_flag[2 * idx + in_side])
		min_cost = min(reach_min_cost[2 * idx + in_side], min_cost);
	    cerr << indent << "min cost is now " << min_cost << endl;

	    reach_flag[2 * idx + in_side] = true;
	    reach_min_cost[2 * idx + in_side] = min_cost;
	}
    }

    stack.pop_back();

    indent.resize(indent.size() - 2);
    cerr << indent << "done " << corridors[idx].name << endl;
    return reach_flag[2 * idx + in_side];
}

//template<typename It>
//pair<int, int> Plan::findEndpointType(
//        ConnectionTypes const& dfs_types,
//        EndpointTypes   const& cost_types,
//        InboundConnections const& inbound_connections,
//        size_t corridor_idx, It begin, It end) const
//{
    //Corridor const& corridor = corridors[corridor_idx];
    //for (It it = begin; it != end; ++it)
    //{
    //    Corridor::Connections::const_iterator conn_it =
    //        corridor.findConnectionFrom(it->center);

    //    if (conn_it == corridor.connections.end())
    //    {
    //        InboundConnections::const_iterator inbound_it = inbound_connections.find(corridor_idx);
    //        if (inbound_it == inbound_connections.end())
    //            continue;

    //        list<Corridor::ConnectionDescriptor> const& inbound = inbound_it->second;
    //        for (conn_it = inbound.begin(); conn_it != inbound.end(); ++conn_it)
    //        {
    //            if (conn_it->get<0>() == it->center)
    //                break;
    //        }

    //        if (conn_it == corridor.connections.end())
    //            continue;
    //    }

    //    ConnectionTypes::const_iterator type_it = dfs_types.find(make_pair(corridor_idx, conn_it->get<1>()));
    //    if (type_it != dfs_types.end())
    //    {
    //        int dfs_type = 0, cost_type = 0;
    //        dfs_type = type_it->second;
    //        if (dfs_type == CONNECTION_BIDIR)
    //            cost_type = cost_types.find( &(*conn_it) )->second;

    //        cerr << "  found type of endpoint " << conn_it->get<0>() << " connected to " << corridors[conn_it->get<1>()].name << endl;
    //        return make_pair(dfs_type, cost_type);
    //    }
    //}
//    throw runtime_error("cannot find type for endpoint");
//}

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
	if (corridors[corridor_idx].isSingleton())
	{
	    cerr << "  " << corridor.name << " is a singleton" << endl;
	    continue;
	}

        // Gather a cost value for each end regions (mean value)
	float costs[2] = { 0, 0 };
	for (PointSet::const_iterator p_it = corridor.end_regions[0].begin(); p_it != corridor.end_regions[0].end(); ++p_it)
            costs[0] += m_nav_function.getValue(p_it->x, p_it->y);
	costs[0] /= corridor.end_regions[0].size();
	for (PointSet::const_iterator p_it = corridor.end_regions[1].begin(); p_it != corridor.end_regions[1].end(); ++p_it)
            costs[1] += m_nav_function.getValue(p_it->x, p_it->y);
	costs[1] /= corridor.end_regions[1].size();

        if (costs[0] > costs[1])
            corridor.reverse();

        corridor.end_types[0] = ENDPOINT_FRONT;
        corridor.end_types[1] = ENDPOINT_BACK;

        cerr << "corridor " << corridor.name << " is oriented FRONT <= BACK as "
            << corridor.voronoi.front().center << " <= " << corridor.voronoi.back().center << endl;
    }
}

void Plan::reorientMedianLines()
{
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor& corridor = corridors[i];
        int front_type = orientations[2 * i];
        int back_type  = orientations[2 * i + 1];
        if (front_type == ENDPOINT_BACK && back_type == ENDPOINT_FRONT)
        {
            cerr << corridor.name << " is oriented BACK[" << corridor.voronoi.front().center << "] => FRONT[" << corridor.voronoi.back().center << "]" << endl;
        }
        else if (front_type == ENDPOINT_FRONT && back_type == ENDPOINT_BACK)
        {
            corridor.reverse();
            cerr << corridor.name << " is oriented FRONT[" << corridor.voronoi.front().center << "] <= BACK[" << corridor.voronoi.back().center << "]" << endl;
        }
        else if (corridor.bidirectional)
            cerr << corridor.name << " is bidirectional" << endl;
        else
        {
            cerr << "error in orientation for " << corridor.name << endl;
            cerr << "  front_type == " << (front_type == ENDPOINT_FRONT ? "FRONT" : "BACK") << endl;
            cerr << "  back_type  == " << (back_type == ENDPOINT_FRONT ? "FRONT" : "BACK") << endl;
        }

        if (!corridor.bidirectional)
        {
            corridor.end_types[0] = ENDPOINT_BACK;
            corridor.end_types[1] = ENDPOINT_FRONT;
        }
    }

}

void Plan::removeBackToBackConnections()
{
    size_t start_idx = findStartCorridor();
    size_t end_idx   = findEndCorridor();

    markDirections_cost();

    Corridor const& start_corridor = corridors[start_idx];
    float cost_margin = m_nav_function.getValue(m_start.x, m_start.y) * 0.05;

    reach_flag.resize(2 * corridors.size());
    fill(reach_flag.begin(), reach_flag.end(), false);
    reach_min_cost.resize(2 * corridors.size());
    orientations.resize(2 * corridors.size());
    fill(orientations.begin(), orientations.end(), 0);

    reach_flag[2 * end_idx] = true;
    reach_flag[2 * end_idx + 1] = true;
    reach_min_cost[2 * end_idx] = 0;
    reach_min_cost[2 * end_idx + 1] = 0;

    std::set< boost::tuple<int, PointID, int, PointID> > result;
    for (Corridor::Connections::const_iterator it = start_corridor.connections.begin(); it != start_corridor.connections.end(); ++it)
    {
	int     target_idx = it->get<1>();
	PointID target_p   = it->get<2>();
	cerr << "initializing DFS-based DAG convertion with " << corridors[target_idx].name << " " << target_p << endl;
	int     target_side = corridors[target_idx].findSideOf(target_p);

        orientations[target_idx * 2 + target_side] = ENDPOINT_BACK;
	result.insert( make_tuple(start_idx, it->get<0>(), target_idx, target_p) );

	vector<int> stack;
	markDirections_DFS(result, stack, target_side, target_idx, end_idx, 0, cost_margin);
    }

    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
	Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator conn_it = connections.begin(),
            end = connections.end();
        while (conn_it != end)
	{
	    if (result.find( make_tuple(corridor_idx, conn_it->get<0>(), conn_it->get<1>(), conn_it->get<2>()) ) == result.end())
		connections.erase(conn_it++);
	    else ++conn_it;
	}
    }

    reorientMedianLines();
}

void Plan::markNullCorridors(vector<int>& useful)
{
    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        list<PointSet> end_regions = corridor.endRegions();

        if (useful[corridor_idx] == USEFUL || end_regions.size() > 1)
            continue;
        else if (corridor.connections.size() <= 1)
            useful[corridor_idx] = NOT_USEFUL;
        else
        {
            set< pair<int, int> > seen;

            // Just blindly create connections between corridors that are
            // connected through this one, and mark this corridor as not useful.
            Corridor::Connections& connections = corridor.connections;
            for(Corridor::Connections::const_iterator conn_a = connections.begin();
                    conn_a != connections.end(); ++conn_a)
            {
                int idx_a        = conn_a->get<1>();
                Corridor& corr_a = corridors[idx_a];

                Corridor::Connections::const_iterator conn_b = conn_a;
                for(++conn_b; conn_b != connections.end(); ++conn_b)
                {
                    int idx_b = conn_b->get<1>();
                    if (idx_a == idx_b)
                        continue;

                    if (seen.count( make_pair(idx_a, idx_b) ))
                        continue;

                    seen.insert( make_pair(idx_a, idx_b) );
                    seen.insert( make_pair(idx_b, idx_a) );

                    // Check that there is no connections between a and b
                    // already
                    if (corr_a.isConnectedTo(idx_b))
                        continue;

                    // Now, create new connections and register in +seen+
                    Corridor& corr_b = corridors[idx_b];
                    corr_a.addConnection(conn_a->get<2>(), idx_b, conn_b->get<2>());
                    corr_b.addConnection(conn_b->get<2>(), idx_a, conn_a->get<2>());
                }
            }

            useful[corridor_idx] = NOT_USEFUL;
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

    std::cerr << "corridor " << corridors[owner].name << " is closest to " << endp << " with d == " << min_distance << std::endl;
    return owner;
}

void Plan::markUselessCorridors(vector<int>& useful)
{
    // Now, do a depth-first search. The useful corridors are the ones that
    // helps connecting an endpoint corridor to another endpoint corridor
    set<int> dfs_stack;

    vector<int> original_useful = useful;
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        if (original_useful[i] == USEFUL)
        {
            dfs_stack.clear();
            markNextCorridors(dfs_stack, i, useful);
        }
    }

    for (size_t i = 0; i < useful.size(); ++i)
    {
        if (useful[i] == 0)
            useful[i] = NOT_USEFUL;
    }

}

void Plan::removeUselessCorridors(vector<int>& useful)
{
    // Now remove the not useful corridors
    for (int i = corridors.size() - 1; i >= 0; --i)
    {
        if (useful[i] == NOT_USEFUL)
        {
            //cerr << "  corridor " << i << " is not useful" << endl;
            removeCorridor(i);
            useful.erase(useful.begin() + i);
        }
        //else if (useful[i] == USEFUL)
        //    //cerr << "  corridor " << i << " is useful" << endl;
        //else
        //    //cerr << "  corridor " << i << " is undetermined" << endl;
    }
}

void Plan::mergeSimpleCrossroads_directed()
{
    vector<int> in_connectivity(corridors.size(), 0);
    vector<int> out_connectivity(corridors.size(), 0);

    for (size_t i = 0; i < corridors.size(); ++i)
    {
        set<int> connectivity = corridors[i].connectivity();
        for (set<int>::const_iterator it = connectivity.begin();
                it != connectivity.end(); ++it)
            in_connectivity[*it]++;

        out_connectivity[i] = connectivity.size();
    }

    for (int i = corridors.size() - 1; i >= 0; --i)
    {
        if (out_connectivity[i] == 0 && in_connectivity[i] == 0)
        {
            removeCorridor(i);
            in_connectivity.erase(in_connectivity.begin() + i);
            out_connectivity.erase(out_connectivity.begin() + i);
        }
    }

    for (int i = 0; i < (int)corridors.size(); ++i)
    {
	if (corridors[i].isSingleton()) continue;
	if (out_connectivity[i] != 1) continue;

        int target_idx = corridors[i].connections.front().get<1>();
        if (corridors[target_idx].isSingleton()) continue;
        if (in_connectivity[target_idx] > 1) continue;

        if (corridors[i].bidirectional || corridors[target_idx].bidirectional)
            continue;

        Corridor& source = corridors[i];
        Corridor& target = corridors[target_idx];
        cerr << "merging " << target.name << " + " << source.name << " out=" << out_connectivity[i] << " in=" << in_connectivity[target_idx] << endl;
        target.merge(source);
        moveConnections(target_idx, i);
        removeCorridor(i);

	in_connectivity[target_idx] = in_connectivity[i];
        in_connectivity.erase(in_connectivity.begin() + i);
        out_connectivity.erase(out_connectivity.begin() + i);
        --i;
    }
}

void Plan::mergeSimpleCrossroads()
{
    // Finally, remove crossroads that connects only two corridors together (by
    // contrast with those that connect more, which are obviously real
    // crossroads).
    list<PointSet> connection_zones;
    map<PointID, int> ownerships;
    PointSet seen;
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor::Connections const& connections = corridors[i].connections;
        Corridor::Connections::const_iterator conn_it;
        for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
        {
            size_t target_idx = conn_it->get<1>();
            if (target_idx < i)
                continue; // already done (connections are symmetric)

            PointID source = conn_it->get<0>();
            PointID target = conn_it->get<2>();
            //cerr << source << " " << target << endl;
            ownerships[source] = i;
            ownerships[target] = target_idx;

            list<PointSet>::iterator source_set = find_if(connection_zones.begin(), connection_zones.end(),
                    bind(&PointSet::count, _1, source));
            list<PointSet>::iterator target_set = find_if(connection_zones.begin(), connection_zones.end(),
                    bind(&PointSet::count, _1, target));

            if (source_set == connection_zones.end())
            {
                if (target_set == connection_zones.end())
                {
                    PointSet new_set;
                    new_set.insert(source);
                    new_set.insert(target);
                    connection_zones.push_back(new_set);
                }
                else
                    target_set->insert(source);
            }
            else if (target_set == connection_zones.end())
                source_set->insert(target);
            else if (source_set != target_set)
            {
                source_set->insert(target_set->begin(), target_set->end());
                connection_zones.erase(target_set);
            }
        }
    }

    //cerr << connection_zones.size() << " crossroads found" << endl;

    // Now that we have clustered the connection points, merge the corridors
    // which are connected by a simple crossroad
    list<PointSet>::const_iterator zone_it;
    for (zone_it = connection_zones.begin(); zone_it != connection_zones.end(); ++zone_it)
    {
        //cerr << *zone_it << endl;

        PointSet const& points = *zone_it;
        set<int> connected;
        for (PointSet::const_iterator p_it = points.begin(); p_it != points.end(); ++p_it)
        {
            connected.insert(ownerships[*p_it]);
            if (connected.size() > 2)
                break;
        }
        if (connected.size() == 2)
        {
            // That is a simple crossroad. Merge the two corridors, update the
            // connections and don't forget to update the ownerships as well --
            // needed for the suite of this loop.
            int into_idx = *connected.begin();
            int from_idx = *(++connected.begin());
            //cerr << "simple: " << into_idx << " and " << from_idx << endl;
            Corridor& into = corridors[into_idx];
            Corridor& from = corridors[from_idx];
            into.merge(from);

            // Remove the connections in +into+ that link to +from+
            { Corridor::Connections& connections = from.connections;
                Corridor::Connections::iterator conn_it;
                for (conn_it = connections.begin(); conn_it != connections.end(); )
                {
                    if (conn_it->get<1>() == from_idx)
                        conn_it = connections.erase(conn_it);
                    else ++conn_it;
                }
            }

            // Update the ownership of points that are in \c from
            {
                map<PointID, int>::iterator owner_it;
                for (owner_it = ownerships.begin(); owner_it != ownerships.end(); ++owner_it)
                {
                    int target_idx = owner_it->second;
                    if (target_idx == from_idx)
                        owner_it->second = into_idx;
                    else if (target_idx > from_idx)
                        --owner_it->second;
                }
            }

            // Move the connections of +target+ into +source+. This removes any
            // connections that may exist between the two.
            moveConnections(into_idx, from_idx);
            removeCorridor(from_idx);
        }
    }
}

int Plan::markNextCorridors(set<int>& stack, int corridor_idx, vector<int>& useful) const
{
    Corridor const& c = corridors[corridor_idx];
    Corridor::Connections::const_iterator conn_it;

    set<int>::iterator stack_it = stack.insert(corridor_idx).first;
    size_t not_useful = 0;
    for (conn_it = c.connections.begin(); conn_it != c.connections.end(); ++conn_it)
    {
        int target_idx = conn_it->get<1>();

        int child_type;
        if (stack.count(target_idx))
            child_type = useful[target_idx];
        else if (useful[target_idx] == NOT_USEFUL)
            child_type = NOT_USEFUL;
        else if (useful[target_idx] == USEFUL)
            child_type = USEFUL;
        else
            child_type = markNextCorridors(stack, target_idx, useful);

        if (child_type == USEFUL)
        {
            cerr << corridor_idx << " is useful" << endl;
            useful[corridor_idx] = USEFUL;
        }
        else if (child_type == NOT_USEFUL)
            ++not_useful;
    }

    if (not_useful == c.connections.size())
    {
        cerr << corridor_idx << " is not useful" << endl;
        useful[corridor_idx] = NOT_USEFUL;
    }
    if (useful[corridor_idx] == 0)
        cerr << corridor_idx << " is left undetermined" << endl;


    stack.erase(stack_it);
    return useful[corridor_idx];
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
	//	cerr << " " << it->center << endl;

	//    cerr << end_regions << " end regions for corridor " << corridor.name << endl;
	//}

        Corridor::Connections const& connections = corridor.connections;
        for (Corridor::Connections::const_iterator it = connections.begin();
                it != connections.end(); ++it)
        {
            size_t target_idx = it->get<1>();
            if (target_idx == i)
                cerr << "  " << corridor.name << " is looping on itself" << endl;

            if (target_idx < 0 || target_idx >= corridors.size())
                cerr << "  wrong target index in connection from " << corridor.name << ": " << target_idx << endl;
        }
    }
}

ostream& nav::operator << (ostream& io, Plan const& plan)
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

