#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>
#include <algorithm>

const int Nav::Plan::USEFUL;
const int Nav::Plan::NOT_USEFUL;
#include <CGAL/Cartesian.h>
#include <CGAL/convex_hull_2.h>

using namespace std;
using namespace Nav;

using boost::mem_fn;
using boost::bind;

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
    set<PointID> seen;
    set<PointID> result;
    typedef CGAL::Point_2<CGAL::Cartesian<int> > Point_2;

    for (size_t corridor_idx = 1; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor const& corridor = corridors[corridor_idx];
        for (Corridor::Connections::const_iterator it = corridor.connections.begin();
                it != corridor.connections.end(); ++it)
        {
            result.clear();

            PointID source = it->get<0>();
            if (seen.count(source))
                continue;

            seen.insert(source);
            findAdjacentBorders(source, corridor, seen, result);

            if (result.empty())
                continue;

            vector<Point_2> unordered_points;
            for (PointSet::const_iterator it = result.begin(); it != result.end(); ++it)
                unordered_points.push_back( Point_2(it->x, it->y) );
            vector<Point_2> hull_points;
            convex_hull_2(unordered_points.begin(), unordered_points.end(), back_inserter(hull_points));

            cerr << " == Crossroad\n";
            for (vector<Point_2>::const_iterator it = hull_points.begin(); it != hull_points.end(); ++it)
                cerr << it->x() << " " << it->y() << "\n";
            cerr << endl;
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

void Plan::simplify(PointSet endpoints)
{
    removeUselessCorridors(endpoints);
    mergeSimpleCrossroads();
}

void Plan::removeUselessCorridors(PointSet endpoints)
{
    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);

    useful_corridors[0] = USEFUL;

    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor& corridor = corridors[i];
        cerr << "  corridor " << i << " has " << corridor.endpointCount() << " endpoints" << endl;
        if (corridor.median.size() == 1)
            useful_corridors[i] = NOT_USEFUL;
    }

    while (!endpoints.empty())
    {
        PointID endp = *endpoints.begin();
        endpoints.erase(endpoints.begin());
        
        for (size_t i = 0; i < corridors.size(); ++i)
        {
            Corridor& corridor = corridors[i];
            if (corridor.contains(endp))
            {
                cerr << "  corridor " << i << " contains " << endp << endl;
                useful_corridors[i] = USEFUL;
            }
        }
    }

    // Now, do a depth-first search. The useful corridors are the ones that
    // helps connecting an endpoint corridor to another endpoint corridor
    vector<int> dfs_stack;
    for (size_t i = 1; i < corridors.size(); ++i)
    {
        if (useful_corridors[i] == USEFUL)
        {
            dfs_stack.clear();
            dfs_stack.push_back(i);
            markNextCorridors(dfs_stack, i, useful_corridors);
        }
    }

    // Now remove the not useful corridors
    for (size_t i = corridors.size() - 1; i > 0; --i)
    {
        if (useful_corridors[i] == NOT_USEFUL)
        {
            cerr << "corridor " << i << " is not useful" << endl;
            removeCorridor(i);
        }
        else if (useful_corridors[i] == USEFUL)
            cerr << "corridor " << i << " is useful" << endl;
        else
            cerr << "corridor " << i << " is undetermined" << endl;
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
            cerr << source << " " << target << endl;
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

    cerr << connection_zones.size() << " crossroads found" << endl;

    // Now that we have clustered the connection points, merge the corridors
    // which are connected by a simple crossroad
    list<PointSet>::const_iterator zone_it;
    for (zone_it = connection_zones.begin(); zone_it != connection_zones.end(); ++zone_it)
    {
        cerr << *zone_it << endl;

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
            cerr << "simple: " << into_idx << " and " << from_idx << endl;
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

