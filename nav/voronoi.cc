#include "voronoi.hh"
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>
#include <stdexcept>
#include <numeric>

#include <iostream>

using namespace std;
using namespace boost;
using namespace Nav;

list<PointSet>::iterator Nav::updateConnectedSets(std::list<PointSet>& sets, PointID p)
{
    list<PointSet>::iterator it;
    for (it = sets.begin(); it != sets.end(); ++it)
    {
        if (p.isNeighbour(*it))
            break;
    }

    if (it == sets.end())
    { // not connected to any set
        sets.push_back( PointSet() );
        sets.rbegin()->insert( p );
        return --sets.end();
    }

    
    list<PointSet>::iterator added_in = it;
    PointSet& adjacent_border = *added_in;
    adjacent_border.insert(p);

    ++it;
    while (it != sets.end())
    {
        if (p.isNeighbour(*it))
        {
            adjacent_border.insert(it->begin(), it->end());
            sets.erase(it++);
        }
        else ++it;
    }

    return added_in;
}

void MedianPoint::offset(PointID const& v)
{
    center += v;
    for (list<PointVector>::iterator it = borders.begin(); it != borders.end(); ++it)
    {
        PointVector& border = *it;
        for (size_t i = 0; i < border.size(); ++i)
            border[i] += v;
    }
}

void MedianPoint::addBorderPoint(PointID const& p)
{
    bbox.update(p);

    BorderList::iterator it;
    for (it = borders.begin(); it != borders.end(); ++it)
    {
        if (p.isNeighbour(*it))
            break;
    }

    if (it == borders.end())
    {
        borders.push_back( PointVector() );
        borders.rbegin()->push_back( p );
    }
    else
    {
        PointVector& adjacent_border = *(it++);
        if (find(adjacent_border.begin(), adjacent_border.end(), p) == adjacent_border.end())
        {
            adjacent_border.push_back(p);
            while (it != borders.end())
            {
                if (p.isNeighbour(*it))
                {
                    adjacent_border.insert(adjacent_border.end(), it->begin(), it->end());
                    borders.erase(it++);
                }
                else ++it;
            }
        }
    }
}

void MedianPoint::mergeBorders(MedianPoint const& p)
{
    for (BorderList::const_iterator border = p.borders.begin(); border != p.borders.end(); ++border)
        for_each(border->begin(), border->end(), bind( mem_fn(&MedianPoint::addBorderPoint), this, _1));
}

bool MedianPoint::isBorderAdjacent(PointID const& p) const
{
    BorderList::const_iterator adjacent_border = find_if(borders.begin(), borders.end(), boost::bind( boost::mem_fn(&PointID::isNeighbour<PointVector>), p, _1));
    return (adjacent_border != borders.end());
}


bool MedianPoint::isBorderAdjacent(MedianPoint const& p) const
{
    if (!bbox.isAdjacent(p.bbox))
        return false;

    for (BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
    {
        PointVector::const_iterator point;
        for (point = border->begin(); point != border->end(); ++point)
        {
            if (p.isBorderAdjacent(*point))
                break;
        }
        if (point == border->end())
            return false;
    }
    return true;
}

bool MedianPoint::isSingleton() const { return borders.size() == 0; }

Point<float> MedianPoint::direction() const
{
    if (borders.size() == 0)
        return Point<float>();
    if (borders.size() != 2)
        throw std::runtime_error("found more than two borders in MedianPoint::direction()");

    BorderList::const_iterator it = borders.begin();
    PointID p1 = accumulate(it->begin(), it->end(), PointID()) / it->size();
    ++it;
    PointID p2 = accumulate(it->begin(), it->end(), PointID()) / it->size();

    return (p2 - p1).normalize();
}

bool MedianPoint::operator == (MedianPoint const& other) const
{
    return width == other.width && borders == other.borders;
}

ostream& Nav::operator << (ostream& io, MedianPoint const& p)
{
    typedef std::list< PointVector > BorderList;
    for (BorderList::const_iterator it = p.borders.begin(); it != p.borders.end(); ++it)
    {
        io << " (";
        for (PointVector::const_iterator p = it->begin(); p != it->end(); ++p)
            io << " " << *p;
        io << ")";
    }
    return io;
}

static void displayPoint(ostream& io, MedianLine const& median_set, PointSet const& border_set, int x, int y)
{
    if (x < -9)
        io << "   ";
    else
        io << "  ";

    for (MedianLine::const_iterator it = median_set.begin(); it != median_set.end(); ++it)
        if (it->center == PointID(x, y))
        {
            io << 1;
            return;
        }

    if (border_set.count(PointID(x, y)))
        io << 0;
    else
        io << "-";
}

void Nav::displayMedianLine(ostream& io, MedianLine const& skel, int xmin, int xmax, int ymin, int ymax)
{
    // Build the set of points that are in the border
    PointSet border_all;
    for (MedianLine::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        MedianPoint::BorderList const& borders = it->borders;
        for (MedianPoint::BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
            border_all.insert(border->begin(), border->end());
    }

    io << "  Bitmap:" << endl;
    io << "     ";
    for (int x = xmin; x < xmax; ++x)
        io << " " << std::setw(2) << x;
    for (int y = ymin; y < ymax; ++y)
    {
        io << endl << "  ";
        io << std::setw(3) << y;
        for (int x = xmin; x < xmax; ++x)
            displayPoint(io, skel, border_all, x, y);
    }

    io << endl << "  Parent list:";
    for (MedianLine::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        io << endl;
        io << "    " << *it << endl;
    }
    io << endl;
}

Corridor::Corridor()
    : bidirectional(false)
{
    end_types[0] = end_types[1] = 0;
}

Corridor Corridor::singleton(PointID const& p, std::string const& name)
{
    Corridor c;
    c.name = name;

    MedianPoint m;
    m.center = p;
    m.bbox.update(p);
    c.median.push_back(m);

    c.center = p;
    c.end_regions[0].insert(p);
    c.bbox.update(p);
    c.median_bbox.update(p);
    return c;
}

template<typename _It>
_It findNearest(_It begin, _It end, PointID const& p)
{
    float min_distance = -1;
    _It result;
    for (_It it = begin; it != end; ++it)
    {
        float d = p.distanceTo2(it->center);
        if (min_distance == -1 || min_distance > d)
        {
            min_distance = d;
            result = it;
        }
    }
    return result;
}

MedianLine::iterator Corridor::findNearestMedian(PointID const& p)
{ return findNearest(median.begin(), median.end(), p); }
MedianLine::const_iterator Corridor::findNearestMedian(PointID const& p) const
{ return findNearest(median.begin(), median.end(), p); }

int Corridor::findSideOf(PointID const& p) const
{
    if (end_regions[0].count(p))
	return 0;
    else if (end_regions[1].count(p))
	return 1;
    else
    {
	cerr << name << " " << p << " is not in an end region" << endl;
	throw runtime_error("given point is not in one of the end regions");
    }
}

void Corridor::add(PointID const& p, MedianPoint const& descriptor, bool ordered)
{
    if (descriptor.borders.size() != 2)
    {
        //throw std::runtime_error("cannot add a point with a connectivity not 2 in a corridor");
        return;
    }

    MedianLine::iterator insert_place = median.end();
    if (median.size() <= 1)
	insert_place = median.end();
    else
    {
	PointID front_point = median.front().center;
	PointID back_point  = median.back().center;
	bool front_n  = front_point.isNeighbour(p);
	bool back_n   = back_point.isNeighbour(p);

        if (ordered)
            insert_place = median.end();
	else if (front_n && back_n)
	{
	    float d_front = p.distance2(front_point);
	    float d_back  = p.distance2(back_point);
	    if (d_front < d_back)
		insert_place = median.begin();
	    else
		insert_place = median.end();
	}
        else if (front_n)
            insert_place = median.begin();
        else if (back_n)
            insert_place = median.end();
        else
        {
            MedianLine::iterator next_front = ++median.begin();
            MedianLine::iterator next_back  = --(--median.end());
            if (next_front->center.isNeighbour(p))
            {
                median.push_front(*next_front);
                median.erase(next_front);
                insert_place = median.begin();
            }
            else if (next_back->center.isNeighbour(p))
            {
                median.push_back(*next_back);
                median.erase(next_back);
                insert_place = median.end();
            }
            else
            {
                float d_front = p.distance2(front_point);
                float d_back  = p.distance2(back_point);
                if (d_front < d_back)
                    insert_place = median.begin();
                else
                    insert_place = median.end();
            }
        }
    }

    insert_place = median.insert(insert_place, descriptor);
    insert_place->center = p;
    median_bbox.update(p);
    mergeBorders(descriptor);
}

void Corridor::add(MedianPoint const& p, bool ordered)
{
    add(p.center, p, ordered);
}

bool Corridor::checkConsistency() const
{
    bool result = true;

    // Check that the median line is ordered properly
    PointID last_point = median.front().center;
    for (MedianLine::const_iterator it = median.begin(); it != median.end(); ++it)
    {
        if (!it->center.isNeighbour(last_point))
        {
            result = false;
            cerr << name << ", error in median ordering: " << last_point << " " << it->center << endl;
        }
        last_point = it->center;
    }

    if (!result)
    {
        cerr << name << " median line is:";
        int line_count = 0;
        for (MedianLine::const_iterator median_it = median.begin();
                median_it != median.end(); ++median_it)
        {
            if (++line_count > 5)
            {
                cerr << endl << "    ";
                line_count = 0;
            }
            cerr << " " << median_it->center;
        }
        cerr << endl;
    }

    // Verify that all connection points are in the median line
    Corridor::Connections::const_iterator conn_it;
    for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
    {
        PointID p = conn_it->get<0>();
        if (find_if(median.begin(), median.end(),
                    bind(&MedianPoint::center, _1) == p) == median.end())
        {
            result = false;
            //cerr << "connection point " << p << " is not in the median" << endl;
        }
    }

    return result;
}

template<typename It>
static It findConnectionFrom(It begin, It end, PointID const& p)
{
    for (It it = begin; it != end; ++it)
    {
        if (it->get<0>() == p)
            return it;
    }
    return end;
}

Corridor::Connections::iterator Corridor::findConnectionFrom(PointID const& p)
{ return ::findConnectionFrom(connections.begin(), connections.end(), p); }
Corridor::Connections::const_iterator Corridor::findConnectionFrom(PointID const& p) const
{ return ::findConnectionFrom(connections.begin(), connections.end(), p); }

template<typename It>
static It findConnectionTo(It begin, It end, int target_idx, PointID const& target_p)
{
    for (It it = begin; it != end; ++it)
    {
        if (it->get<1>() == target_idx && it->get<2>() == target_p)
            return it;
    }
    return end;
}

Corridor::Connections::iterator Corridor::findConnectionTo(int target_idx, PointID const& p)
{ return ::findConnectionTo(connections.begin(), connections.end(), target_idx, p); }
Corridor::Connections::const_iterator Corridor::findConnectionTo(int target_idx, PointID const& p) const
{ return ::findConnectionTo(connections.begin(), connections.end(), target_idx, p); }

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

void Corridor::reverse()
{
    reverseList(median);
    swap(end_types[0], end_types[1]);
    swap(end_regions[0], end_regions[1]);
}

void Corridor::addConnection(PointID const& source_p, int target_idx, PointID const& target_p)
{
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->get<1>() == target_idx &&
                it->get<2>() == target_p &&
                it->get<0>() == source_p)
            return;
    }

    connections.push_back( make_tuple(source_p, target_idx, target_p));
}

bool Corridor::isConnectedTo(int other_corridor) const
{
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->get<1>() == other_corridor)
            return true;
    }
    return false;
}

void Corridor::removeConnectionsTo(int other_corridor)
{
    Connections::iterator it = connections.begin();
    while (it != connections.end())
    {
        if (it->get<1>() == other_corridor)
            connections.erase(it++);
        else ++it;
    }
}

void Corridor::merge(Corridor& corridor)
{
    // It is assumed that both corridors are ordered FRONT => BACK. After the
    // merge, the ordering is therefore kept.
    //
    // Still, we need to update the end regions
    PointID this_front = median.front().center;
    PointID this_back  = median.back().center;
    PointID other_front = corridor.median.front().center;
    PointID other_back  = corridor.median.back().center;

    if (bidirectional && corridor.bidirectional)
        throw runtime_error("don't know how to merge bidirectional corridors");
    else if (bidirectional || corridor.bidirectional)
        throw runtime_error("cannot merge a bidirectional corridor with a non-bidirectional one");

    float a_then_b = this_back.distance2(other_front);
    float b_then_a = other_back.distance2(this_front);
 
    for (MedianLine::const_iterator median_it = corridor.median.begin(); median_it != corridor.median.end(); ++median_it)
    {
        mergeBorders(*median_it);
        median_bbox.update(median_it->center);
    }

    if (a_then_b < b_then_a)
        median.splice(median.end(), corridor.median);
    else
        median.splice(median.begin(), corridor.median);

    checkConsistency();
}

bool Corridor::contains(PointID const& p) const
{
    if (!bbox.isIncluded(p))
        return false;

    return true; // buggy ...
}

bool Corridor::isNeighbour(PointID const& p) const
{
    if (!bbox.isNeighbour(p))
        return false;
    if (contains(p))
        return true;
    return true;
}

bool Corridor::isMedianNeighbour(PointID const& p) const
{
    if (!median_bbox.isNeighbour(p))
        return false;

    for (MedianLine::const_iterator it = median.begin(); it != median.end(); ++it)
        if (it->center.isNeighbour(p))
            return true;

    return false;
}

void Corridor::clear()
{
    median_bbox = bbox = BoundingBox();
    borders.clear();
    median.clear();
    connections.clear();
}

bool Corridor::operator == (Corridor const& other) const
{
    return median == other.median;
}

set<int> Corridor::connectivity() const
{
    set<int> result;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
        result.insert(it->get<1>());
    return result;
}

void Corridor::buildEndRegions()
{
    end_regions[0].clear();
    end_regions[1].clear();
    PointID front = median.front().center;
    PointID back  = median.back().center;

    for (Connections::const_iterator conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
    {
	if (name == "60")
	    cerr << conn_it->get<0>() << endl;

	float d_front = front.distance2(conn_it->get<0>());
	float d_back  = back.distance2(conn_it->get<0>());
	if (d_front <= d_back)
	    end_regions[0].insert(conn_it->get<0>());
	else
	    end_regions[1].insert(conn_it->get<0>());
    }
}

list<PointSet> Corridor::endRegions() const
{
    list<PointSet> endpoints;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
        updateConnectedSets(endpoints, it->get<0>());
    return endpoints;
}

PointID Corridor::adjacentEndpoint(PointID const& p) const
{
    for (MedianLine::const_iterator it = median.begin(); it != median.end(); ++it)
    {
        if (it->center.isNeighbour(p))
            return it->center;
    }
    return PointID();
}

ostream& Nav::operator << (ostream& io, Corridor const& corridor)
{
    displayMedianLine(io, corridor.median, corridor.bbox.min.x - 1, corridor.bbox.max.x + 1, corridor.bbox.min.y - 1, corridor.bbox.max.y + 1);

    io << "  Borders:\n";
    io << "    " << static_cast<MedianPoint const&>(corridor);
    io << "\n";
    io << "  Connections:\n";
    for (Corridor::Connections::const_iterator it = corridor.connections.begin(); it != corridor.connections.end(); ++it)
        io << "    " << it->get<0>() << " -> [" << it->get<1>() << ": " << it->get<2>() << "]" << endl;
    return io;
}

