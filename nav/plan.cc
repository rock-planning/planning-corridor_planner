#include "plan.hh"
#include <iostream>
#include <iomanip>
#include <boost/bind.hpp>

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

