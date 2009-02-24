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

const int Nav::Plan::USEFUL;
const int Nav::Plan::NOT_USEFUL;

using namespace std;
using namespace Nav;
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

void Plan::addAdjacentBorders(MedianPoint const& p0, MedianPoint const& p1, set<PointID>& result) const
{
    for (list<PointVector>::const_iterator p0_border_it = p0.borders.begin(); p0_border_it != p0.borders.end(); ++p0_border_it)
    {
        for (PointVector::const_iterator p0_it = p0_border_it->begin(); p0_it != p0_border_it->end(); ++p0_it)
        {
            if (p1.isBorderAdjacent(*p0_it))
                result.insert(*p0_it);
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

pair<PointID, PointID> Plan::split(int corridor_idx, MedianLine::iterator it)
{
    // Now, create two empty corridors whose single median line point are
    // the start and end point. Moreover, we split the start and end
    // corridors and connect them to these "virtual" corridors.
    Corridor& corr = corridors[corridor_idx];

    Corridor new_corr;
    new_corr.median.splice(new_corr.median.end(), corr.median, it, corr.median.end());

    corr.borders.clear();
    new_corr.borders.clear();

    // Generate the border sets and gather all center points in median_point_set
    // to update the connections later on
    map<PointID, int> ownerships;
    for (MedianLine::iterator median_it = new_corr.median.begin(); median_it != new_corr.median.end(); ++median_it)
    {
        ownerships[median_it->center] = 1;
        new_corr.mergeBorders(*median_it);
    }

    for (MedianLine::iterator median_it = corr.median.begin(); median_it != corr.median.end(); ++median_it)
    {
        ownerships[median_it->center] |= 2;
        corr.mergeBorders(*median_it);
    }

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
    return make_pair(corr.median.back().center, new_corr.median.front().center);
}

void Plan::simplify()
{
    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);

    size_t original_count = corridors.size();

    // BIG FAT NOTE: cannot use findStartCorridor() here as the start corridor
    // is not yet created.
    useful_corridors[findCorridorOf(m_start)] = USEFUL;
    useful_corridors[findCorridorOf(m_end)]   = USEFUL;
    markNullCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    markUselessCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    //mergeSimpleCrossroads();
    cerr << "simplification pass" << endl;
    checkConsistency();

    // We changed something. Re-run the simplification process.
    if (original_count != corridors.size())
        simplify();
    else if (!m_nav_function.empty())
    {
        int start_idx = findCorridorOf(m_start);
        MedianLine::iterator it = corridors[start_idx].findNearestMedian(m_start);
        PointID conn_p = it->center;
        pair<PointID, PointID> new_endpoints = split(start_idx, it);
        cerr << "split " << corridors[start_idx].name << ", created " << corridors.back().name << endl;
        Corridor start_corridor = Corridor::singleton(m_start, 
                lexical_cast<string>(corridors.size()) + "_start");
	start_corridor.end_types[0] = ENDPOINT_FRONT;
	size_t new_corridor = corridors.size() - 1;
        start_corridor.addConnection(m_start, start_idx, new_endpoints.first);
	corridors[start_idx].addConnection(new_endpoints.first, corridors.size(), m_start);
        start_corridor.addConnection(m_start, new_corridor, new_endpoints.second);
	corridors[new_corridor].addConnection(new_endpoints.second, corridors.size(), m_start);
	cerr << new_endpoints.second << " " << m_start << endl;
        corridors.push_back(start_corridor);

	if (corridors[start_idx].isSingleton())
	{
	    moveConnections(corridors.size() - 1, start_idx);
	    --new_corridor;
	    removeCorridor(start_idx);
	}
	if (corridors[new_corridor].isSingleton())
	{
	    moveConnections(corridors.size() - 1, new_corridor);
	    removeCorridor(new_corridor);
	}

        int end_idx = findCorridorOf(m_end);
        it = corridors[end_idx].findNearestMedian(m_end);
        conn_p = it->center;
        new_endpoints = split(end_idx, it);
        cerr << "split " << corridors[end_idx].name << ", created " << corridors.back().name << endl;
        Corridor end_corridor = Corridor::singleton(m_end,
                lexical_cast<string>(corridors.size()) + "_end");
	end_corridor.end_types[0] = ENDPOINT_BACK;

	new_corridor = corridors.size() - 1;
        corridors[end_idx].addConnection(new_endpoints.first, corridors.size(), m_end);
	end_corridor.addConnection(m_end, end_idx, new_endpoints.first);
        corridors[new_corridor].addConnection(new_endpoints.second, corridors.size(), m_end);
	end_corridor.addConnection(m_end, new_corridor, new_endpoints.second);
        corridors.push_back(end_corridor);

	if (corridors[end_idx].isSingleton())
	{
	    moveConnections(corridors.size() - 1, end_idx);
	    removeCorridor(end_idx);
	    --new_corridor;
	}
	if (corridors[new_corridor].isSingleton())
	{
	    moveConnections(corridors.size() - 1, new_corridor);
	    removeCorridor(new_corridor);
	}

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
		for (MedianLine::const_iterator median_it = corridor.median.begin();
			median_it != corridor.median.end(); ++median_it)
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

        checkConsistency();

        removeBackToBackConnections();

        cerr << "removed back-to-back connections" << endl;
        checkConsistency();

        //vector<int> useful_corridors;
        //useful_corridors.resize(corridors.size(), 0);
        //useful_corridors[ findStartCorridor() ] = USEFUL;
        //useful_corridors[ findEndCorridor() ] = USEFUL;

        //markUselessCorridors(useful_corridors);
        //removeUselessCorridors(useful_corridors);

        //cerr << "removed useless corridors" << endl;
        //checkConsistency();

        mergeSimpleCrossroads_directed();

        cerr << "merged simple crossroads" << endl;
        //checkConsistency();
    }
}

template<typename T>
void reverseList(list<T>& l)
{
    if (l.size() < 2)
        return;

    typename list<T>::iterator
        end = l.end(),
        it = l.end();

    // Get one element before the last element
    --it; --it;
    while (it != l.begin())
        l.splice(l.end(), l, it--);

    l.splice(l.end(), l, l.begin());
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

    if (reach_flag[2 * idx + in_side] && backwards)
    {
	cerr << indent << corridor.name << " is bidirectional" << endl;
	corridor.bidirectional = true;
    }

    stack.pop_back();

    indent.resize(indent.size() - 2);
    cerr << indent << "done " << corridors[idx].name << endl;
    return reach_flag[2 * idx + in_side];
}

template<typename It>
pair<int, int> Plan::findEndpointType(
        ConnectionTypes const& dfs_types,
        EndpointTypes   const& cost_types,
        InboundConnections const& inbound_connections,
        size_t corridor_idx, It begin, It end) const
{
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
    throw runtime_error("cannot find type for endpoint");
}

void Plan::markDirections_cost(set<int> const& bidir,
        ConnectionTypes const& dfs_types,
        EndpointTypes& endpoint_types)
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

	if (costs[0] < costs[1])
	{
	    corridor.end_types[0] = ENDPOINT_FRONT;
	    corridor.end_types[1] = ENDPOINT_BACK;

	    cerr << "corridor " << corridor.name << " is oriented FRONT <= BACK as "
		<< corridor.median.front().center << " <= " << corridor.median.back().center << endl;
	}
	else
	{
	    corridor.end_types[0] = ENDPOINT_BACK;
	    corridor.end_types[1] = ENDPOINT_FRONT;

	    cerr << "corridor " << corridor.name << " is oriented BACK => FRONT as "
		<< corridor.median.front().center << " => " << corridor.median.back().center << endl;
	}
    }
}

void Plan::reorientMedianLines(ConnectionTypes const& types, EndpointTypes const& endpoint_types,
        InboundConnections const& inbound_connections)
{
//    for (size_t corridor_idx = 0; corridor_idx < corridors.size(); ++corridor_idx)
//    {
//        Corridor& corridor = corridors[corridor_idx];
//        Corridor::Connections& connections = corridor.connections;
//
//        cerr << "looking at corridor " << corridor.name << endl;
//
//        int dfs_front_type  = CONNECTION_UNKNOWN,
//            dfs_back_type   = CONNECTION_UNKNOWN,
//            cost_front_type = CONNECTION_UNKNOWN,
//            cost_back_type  = CONNECTION_UNKNOWN;
//
//        tie(dfs_front_type, cost_front_type) =
//            findEndpointType(types, endpoint_types, inbound_connections, corridor_idx,
//                    corridor.median.begin(), corridor.median.end());
//        cerr << "  front connection: dfs_type=" << dfs_front_type << ", cost_type=" << cost_front_type << endl;
//
//        tie(dfs_back_type, cost_back_type) =
//            findEndpointType(types, endpoint_types, inbound_connections, corridor_idx,
//                    corridor.median.rbegin(), corridor.median.rend());
//        cerr << "  back connection: dfs_type=" << dfs_back_type << ", cost_type=" << cost_back_type << endl;
//
//
//        // Use cost-based determination only when necessary, as the DFS method
//        // is much more robust.
//        int front_type, back_type;
//        if (dfs_front_type == CONNECTION_BIDIR && dfs_back_type == CONNECTION_BIDIR)
//        {
//            front_type = cost_front_type;
//            back_type  = cost_back_type;
//        }
//        else
//        {
//            front_type = dfs_front_type;
//            back_type  = dfs_back_type;
//        }
//
//
//        if (front_type == CONNECTION_UNKNOWN && back_type == CONNECTION_UNKNOWN)
//        {
//            cerr << "  cannot know its orientation (unknown)" << endl;
//            continue;
//        }
//        else if (front_type == CONNECTION_BIDIR && back_type == CONNECTION_BIDIR)
//        {
//            cerr << "  cannot know its orientation (bidir)" << endl;
//            throw runtime_error("unknown orientation");
//            continue;
//        }
//        else if (front_type == back_type)
//        {
//            cerr << "  median: ";
//            int i = 0;
//            for (MedianLine::const_iterator median_it = corridor.median.begin();
//                    median_it != corridor.median.end(); ++median_it)
//            {
//                if (++i > 10)
//                {
//                    cerr << endl << "    ";
//                    i = 0;
//                }
//                cerr << " " << median_it->center;
//            }
//            cerr << endl;
//
//            Corridor::Connections::const_iterator conn_it;
//            cerr << "  connections:" << endl;
//            for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
//            {
//                cerr << "    " << conn_it->get<0>() << " => " << corridors[conn_it->get<1>()].name << " " << conn_it->get<2>() << " of type " << types.find( make_pair(corridor_idx, conn_it->get<1>()) )->second << endl;
//            }
//
//            cerr << "  cannot know its orientation (" << front_type << ")" << endl;
//            throw runtime_error("unknown orientation");
//            continue;
//        }
//        else if (front_type != CONNECTION_BACK_TO_FRONT && back_type != CONNECTION_FRONT_TO_BACK)
//        {
//            cerr << "  inverting order" << endl;
//            reverseList(corridor.median);
//            cerr << "  new order: " << corridor.median.front().center << " " << corridor.median.back().center << endl;
//        }
//    }
}

void Plan::removeBackToBackConnections()
{
    size_t start_idx = findStartCorridor();
    size_t end_idx   = findEndCorridor();

    set<int> bidir; // this set will hold the corridors whose orientation cannot
                    // be determined by the DFS part

    // this map will hold the information we get out of the cost-based method.
    // Are included onlu the endpoints of the corridors that are undetermined
    // after the DFS part
    ConnectionTypes types;
    EndpointTypes endpoint_types;
    markDirections_cost(bidir, types, endpoint_types);

    // First pass: do a DFS and mark the connections in the direction which goes
    // to the goal. Corridors that meet the DFS stack are ignored. If two
    // branches use the same corridor in different directions, mark the
    // associated connections as bidirectional.
    Corridor const& start_corridor = corridors[start_idx];
    float cost_margin = m_nav_function.getValue(m_start.x, m_start.y) * 0.05;

    reach_flag.resize(2 * corridors.size());
    fill(reach_flag.begin(), reach_flag.end(), false);
    reach_min_cost.resize(2 * corridors.size());
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

	//PointSet front_side, back_side;

	//if (corridor.end_types[0] == ENDPOINT_FRONT)
	//    front_side = corridor.end_regions[0];
	//else if (corridor.end_types[1] == ENDPOINT_FRONT)
	//    front_side = corridor.end_regions[1];

	//if (corridor.end_types[0] == ENDPOINT_BACK)
	//    back_side = corridor.end_regions[0];
	//else if (corridor.end_types[1] == ENDPOINT_BACK)
	//    back_side = corridor.end_regions[1];

	//Corridor::Connections& connections = corridor.connections;
        //Corridor::connection_iterator conn_it = connections.begin(),
        //    end = connections.end();
        //while (conn_it != end)
        //{
	//    PointID src_p          = conn_it->get<0>();

        //    if (conn_it->get<1>() == corridor_idx)
        //        connections.erase(conn_it++);
	//    else if (back_side.count(src_p))
	//	connections.erase(conn_it++);
	//    else if (front_side.count(src_p))
	//    {
	//	Corridor const& target = corridors[conn_it->get<1>()];
	//	PointID target_p       = conn_it->get<2>();
	//	int target_side = target.findSideOf(conn_it->get<2>());
	//	int target_type = target.end_types[target_side];
	//	if (target_type == ENDPOINT_FRONT)
	//	    connections.erase(conn_it++);
	//	else ++conn_it;
	//    }
	//    else ++conn_it;
	//}
    }

    //reorientMedianLines(types, endpoint_types, inbound_connections);
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
        if (corridors[owner].isSingleton() && corridors[owner].center == m_start)
            return owner;

    throw runtime_error("no corridor for start point");
}
int Plan::findEndCorridor() const
{
    for (size_t owner = 0; owner < corridors.size(); ++owner)
        if (corridors[owner].isSingleton() && corridors[owner].center == m_end)
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
            continue;

        MedianLine::const_iterator nearest = corridor.findNearestMedian(endp);
        float d = endp.distance2(nearest->center);
        if (min_distance == -1 || min_distance > d)
        {
            min_distance = d;
            owner = i;
            if (min_distance == 0)
                break;
        }
    }
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

    vector<bool> simple_corridor(corridors.size());
    for (size_t i = 0; i < corridors.size(); ++i) 
        simple_corridor[i] = (!corridors[i].isSingleton() && out_connectivity[i] <= 1 && in_connectivity[i] <= 1);

    for (int i = 0; i < (int)corridors.size(); ++i)
    {
	if (corridors[i].isSingleton()) continue;
	if (out_connectivity[i] != 1) continue;

        int target_idx = corridors[i].connections.front().get<1>();
        if (in_connectivity[target_idx] > 1) continue;

        Corridor const& source = corridors[i];
        Corridor& target = corridors[target_idx];
        cerr << "merging " << target.name << " + " << source.name << " out=" << out_connectivity[i] << " in=" << in_connectivity[target_idx] << endl;
        target.merge(source);
        target.name = target.name + "+" + source.name;
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

	//size_t end_regions = corridor.endRegions().size();
	//if (!corridor.isSingleton() && end_regions != 2)
	//{
	//    for (MedianLine::const_iterator it = corridor.median.begin();
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
                cerr << "  " << corridor.name << " is looping on itself" << target_idx << endl;

            if (target_idx < 0 || target_idx >= corridors.size())
                cerr << "  wrong target index in connection from " << corridor.name << ": " << target_idx << endl;
        }
    }
}

ostream& Nav::operator << (ostream& io, Plan const& plan)
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

