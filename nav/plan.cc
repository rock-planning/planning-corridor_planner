#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

// #include <CGAL/Cartesian.h>
// #include <CGAL/convex_hull_2.h>

const int Nav::Plan::USEFUL;
const int Nav::Plan::NOT_USEFUL;

using namespace std;
using namespace Nav;
using namespace boost;

void Plan::clear()
{
    corridors.clear();
}

void Plan::removeCorridor(int idx)
{
    corridors.erase(corridors.begin() + idx);
    for (corridor_iterator corridor = corridors.begin(); corridor != corridors.end(); ++corridor)
    {
        Corridor::Connections& connections = corridor->connections;
        Corridor::connection_iterator it = connections.begin();
        while (it != connections.end())
        {
            if (it->get<1>() == idx)
                connections.erase(it++);
            else
            {
                if (it->get<1>() > idx)
                    it->get<1>()--;
                ++it;
            }
        }
    }
}

void Plan::addAdjacentBorders(MedianPoint const& p0, MedianPoint const& p1, set<PointID>& result) const
{
    for (list<PointSet>::const_iterator p0_border_it = p0.borders.begin(); p0_border_it != p0.borders.end(); ++p0_border_it)
    {
        for (PointSet::const_iterator p0_it = p0_border_it->begin(); p0_it != p0_border_it->end(); ++p0_it)
        {
            if (p1.isBorderAdjacent(*p0_it))
                result.insert(*p0_it);
        }
    }
}

/** Adds to +result+ the set of connections starting from +p+ in +corridor+ */
void Plan::findAdjacentBorders(PointID p, Corridor const& corridor,
        set<PointID>& seen, set<PointID>& result) const
{
    MedianPoint const& median_point = corridor.median.find(p)->second;

    for (Corridor::Connections::const_iterator it = corridor.connections.begin(); it != corridor.connections.end(); ++it)
    {
        if (it->get<0>() == p)
        {
            int target_idx = it->get<1>();
            PointID target = it->get<2>();
            if (target_idx != 0 && !seen.count(target))
            {
                seen.insert(target);
                addAdjacentBorders(median_point, corridors[target_idx].median.find(target)->second, result);
                findAdjacentBorders(target, corridors[target_idx], seen, result);
            }
        }
    }
}

void Plan::buildCrossroads() const
{
//    set<PointID> seen;
//    set<PointID> result;
//    typedef CGAL::Point_2<CGAL::Cartesian<int> > Point_2;
//
//    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
//    {
//        Corridor const& corridor = corridors[corridor_idx];
//        for (Corridor::Connections::const_iterator it = corridor.connections.begin();
//                it != corridor.connections.end(); ++it)
//        {
//            result.clear();
//
//            PointID source = it->get<0>();
//            if (seen.count(source))
//                continue;
//
//            seen.insert(source);
//            findAdjacentBorders(source, corridor, seen, result);
//
//            if (result.empty())
//                continue;
//
//            vector<Point_2> unordered_points;
//            for (PointSet::const_iterator it = result.begin(); it != result.end(); ++it)
//                unordered_points.push_back( Point_2(it->x, it->y) );
//            vector<Point_2> hull_points;
//            convex_hull_2(unordered_points.begin(), unordered_points.end(), back_inserter(hull_points));
//
//            cerr << " == Crossroad\n";
//            for (vector<Point_2>::const_iterator it = hull_points.begin(); it != hull_points.end(); ++it)
//                cerr << it->x() << " " << it->y() << "\n";
//            cerr << endl;
//        }
//    }
}

void Plan::concat(Plan const& other)
{
    int merge_start = corridors.size();
    copy(other.corridors.begin(), other.corridors.end(), back_inserter(corridors));

    for (vector<Corridor>::iterator it = corridors.begin() + merge_start; it != corridors.end(); ++it)
        for (Corridor::connection_iterator c = it->connections.begin(); c != it->connections.end(); ++c)
            c->get<1>() += merge_start;
}

void Plan::moveConnections(int into_idx, int from_idx)
{
    Corridor::Connections& from = corridors[from_idx].connections;
    Corridor::Connections& into = corridors[into_idx].connections;
    Corridor::Connections::iterator conn_it;
    for (conn_it = from.begin(); conn_it != from.end(); ++conn_it)
    {
        int target_idx = conn_it->get<1>();
        if (target_idx == into_idx)
            continue;

        into.push_back(*conn_it);
        Corridor::Connections& target = corridors[target_idx].connections;
        Corridor::Connections::iterator target_it;
        for (target_it = target.begin(); target_it != target.end(); ++target_it)
        {
            if (target_it->get<1>() == from_idx)
                target_it->get<1>() = into_idx;
        }
    }
}

void Plan::simplify(PointID start, PointID end)
{
    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);
    useful_corridors[0] = USEFUL;

    size_t original_count = corridors.size();

    markEndpointCorridors(useful_corridors, start, end);
    markNullCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);
    markUselessCorridors(useful_corridors);
    removeUselessCorridors(useful_corridors);

    mergeSimpleCrossroads();

    // We changed something. Re-run the simplification process.
    if (original_count != corridors.size())
        simplify(start, end);
}

void Plan::simplify(PointID start, PointID end, GridGraph const& navmap)
{
    simplify(start, end);
    removeBackToBackConnections(start, end, navmap);
}

void Plan::removeBackToBackConnections(PointID start, PointID end, GridGraph const& navmap)
{
    // Mark, for each corridor, which endpoints are "front" and which are "back"
    // based on the cost. Note that endpoint corridors have either all front or
    // all back (based on wether they are start or end points).

    static const int FRONT_LINE = 0;
    static const int BACK_LINE = 1;
    size_t start_idx = findEndpointCorridor(start);
    size_t end_idx   = findEndpointCorridor(end);

    typedef map< pair<int, PointID>, int> EndpointTypemap; 
    EndpointTypemap types;

    {
        Corridor& corridor = corridors[start_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();
        for (; it != end; ++it)
            types[ make_pair(start_idx, it->get<0>()) ] = FRONT_LINE;
    }
    {
        Corridor& corridor = corridors[end_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();
        for (; it != end; ++it)
            types[ make_pair(end_idx, it->get<0>()) ] = BACK_LINE;
    }

    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        if (corridor_idx == start_idx || corridor_idx == end_idx)
            continue;

        Corridor& corridor = corridors[corridor_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();

        // Gather a cost value for the endpoints of each existing connections
        list< tuple<int, PointID, float> > endpoint_costs;
        float min_value = std::numeric_limits<float>::max(),
              max_value = std::numeric_limits<float>::min();
        for (; it != end; ++it)
        {
            PointID target_p = it->get<2>();
            float value = navmap.getValue(target_p.x, target_p.y);

            endpoint_costs.push_back(make_tuple( corridor_idx, it->get<0>(), value));
            min_value = min(value, min_value);
            max_value = max(value, max_value);
        }

        float min_bound = (max_value +  min_value) / 2;
        float max_bound = (max_value + min_value) / 2;
        for (list< tuple<int, PointID, float> >::const_iterator it = endpoint_costs.begin(); it != endpoint_costs.end(); ++it)
        {
            float cost = it->get<2>();
            cerr << min_bound << " " << cost << " " << max_bound << " " << it->get<1>() << endl;
            if (cost < min_bound)
                types[ make_pair(it->get<0>(), it->get<1>()) ] = FRONT_LINE;
            else if (cost > max_bound)
                types[ make_pair(it->get<0>(), it->get<1>()) ] = BACK_LINE;
            else
            {
                cerr << "min_bound = " << min_bound << ", max_bound = " << max_bound << ", cost = " << cost << endl;
                throw std::runtime_error("cost in [min_bound, max_bound]");
            }
        }
    }

    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        Corridor::Connections& connections = corridor.connections;
        Corridor::connection_iterator it = connections.begin(),
            end = connections.end();

        while (it != end)
        {
            PointID endpoint = it->get<0>();
            int type = types[ make_pair(corridor_idx, endpoint) ];

            if (type == FRONT_LINE)
                cerr << corridor_idx << " " << endpoint << " is on the front line" << endl;
            else
                cerr << corridor_idx << " " << endpoint << " is on the back line" << endl;

            if (type == BACK_LINE)
                connections.erase(it++);
            else
            {
                // Check the direction of the other endpoint. If it is also
                // front, then remove
                int target_idx = it->get<1>();
                PointID target_endpoint = it->get<2>();
                if (types[ make_pair(target_idx, target_endpoint) ] == FRONT_LINE)
                    connections.erase(it++);
                else
                    ++it;
            }
        }
    }
}

void Plan::markNullCorridors(vector<int>& useful)
{
    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        list<PointSet> end_regions = corridor.endRegions();

        if (useful[corridor_idx] == USEFUL || end_regions.size() > 1)
            continue;
        else if (corridor.connections.size() <= 1)
        {
            //cerr << "  " << corridor_idx << " is a null corridor" << endl;
            useful[corridor_idx] = NOT_USEFUL;
        }
        else
        {
            // We still have to check that there is no two corridors that
            // are connected only through this one, i.e. that we don't have
            //
            // A => B => C
            //
            // but no direct A => C connection, in which case we can't remove B
            Corridor::Connections& connections = corridor.connections;
            Corridor::Connections::const_iterator conn_it = connections.begin();
            while (conn_it != connections.end())
            {
                int conn_idx = conn_it->get<1>();
                Corridor& conn_corridor = corridors[conn_idx];

                Corridor::Connections::const_iterator target_it = connections.begin();
                for (; target_it != connections.end(); ++target_it)
                {
                    int target_idx = target_it->get<1>();
                    if (target_idx == conn_idx)
                        continue;
                    if (!conn_corridor.isConnectedTo(target_it->get<1>()))
                        break;
                }

                // increment first, since we may invalidate the iterator by
                // removing the connections
                if (target_it == connections.end())
                {
                    // we can remove this connection
                    corridor.removeConnectionsTo(conn_idx);
                    conn_corridor.removeConnectionsTo(corridor_idx);

                    // we don't know where is the next iterator
                    // (removeConnectionsTo can remove more than one
                    // connection). Just start again ...
                    conn_it = connections.begin();
                }
                else ++conn_it;
            }

            if (corridor.connections.size() <= 1)
            {
                //cerr << "  " << corridor_idx << " is a null corridor" << endl;
                useful[corridor_idx] = NOT_USEFUL;
            }
        }
    }
}

int Plan::findEndpointCorridor(PointID const& endp)
{
    float min_distance = -1;
    int owner = -1;

    for (size_t i = 1; i < corridors.size(); ++i)
    {
        Corridor& corridor = corridors[i];
        if (!corridor.bbox.isNeighbour(endp))
            continue;

        for (MedianLine::const_iterator median_it = corridor.median.begin(); median_it != corridor.median.end(); ++median_it)
        {
            float d = endp.distanceTo2(median_it->first);
            if (min_distance == -1 || min_distance > d)
            {
                min_distance = d;
                owner = i;
            }
        }
    }
    return owner;
}

void Plan::markEndpointCorridors(vector<int>& useful, PointID start, PointID end)
{
    // Mark as useful the corridors that contain endpoints. What we assume here
    // is that there is at most one corridor which contains the endpoints.
    PointID endpoints[2] = { start, end };
    for (int endp_idx = 0; endp_idx < 2; ++endp_idx)
    {
        PointID endp = endpoints[endp_idx];
        int owner = findEndpointCorridor(endp);

        if (owner == -1)
            throw std::runtime_error("no owner for endpoint " + lexical_cast<string>(endp));

        useful[owner] = USEFUL;
    }
}

void Plan::markUselessCorridors(vector<int>& useful)
{
    // Now, do a depth-first search. The useful corridors are the ones that
    // helps connecting an endpoint corridor to another endpoint corridor
    vector<int> dfs_stack;
    for (size_t i = 1; i < corridors.size(); ++i)
    {
        if (useful[i] == USEFUL)
        {
            dfs_stack.clear();
            dfs_stack.push_back(i);
            markNextCorridors(dfs_stack, i, useful);
        }
    }
}

void Plan::removeUselessCorridors(vector<int>& useful)
{
    // Now remove the not useful corridors
    for (size_t i = corridors.size() - 1; i > 0; --i)
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

void Plan::mergeSimpleCrossroads()
{
    // Finally, remove crossroads that connects only two corridors together (by
    // contrast with those that connect more, which are obviously real
    // crossroads).
    list<PointSet> connection_zones;
    map<PointID, int> ownerships;
    PointSet seen;
    for (size_t i = 1; i < corridors.size(); ++i)
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

            // Move the connections of +target+ into +source+. This
            // does not copy the connections between the two and updates the
            // other corridors
            moveConnections(into_idx, from_idx);

            // And now remove
            removeCorridor(from_idx);
        }
    }
}

int Plan::markNextCorridors(vector<int>& stack, int corridor_idx, vector<int>& useful) const
{
    Corridor const& c = corridors[corridor_idx];
    Corridor::Connections::const_iterator conn_it;

    int is_useful = (useful[corridor_idx] == USEFUL ? USEFUL : NOT_USEFUL);
    for (conn_it = c.connections.begin(); conn_it != c.connections.end(); ++conn_it)
    {
        int target_idx = conn_it->get<1>();
        if (target_idx == 0 || *stack.rbegin() == target_idx)
            continue;

        // If the target is in the stack, it means we don't know yet if this
        // connection is useful or not (it will depend on wether target_idx is
        // useful or not).  Stay undetermined after this visit.
        if (find(stack.begin(), stack.end(), target_idx) != stack.end())
        {
            if (is_useful == NOT_USEFUL)
                is_useful = 0;
            continue;
        }

        int target_useful = useful[target_idx];
        if (!target_useful)
        {
            stack.push_back(corridor_idx);
            target_useful = markNextCorridors(stack, target_idx, useful);
            stack.pop_back();
        }

        if (target_useful == USEFUL)
            is_useful = USEFUL;
    }

    return (useful[corridor_idx] = is_useful);
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

