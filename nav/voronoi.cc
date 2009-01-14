#include "voronoi.hh"
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>
#include <stdexcept>
#include <numeric>

#include <iostream>

using boost::bind;
using boost::mem_fn;
using namespace std;
using namespace Nav;

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
        borders.push_back( PointSet() );
        borders.rbegin()->insert( p );
    }
    else
    {
        PointSet& adjacent_border = *(it++);
        adjacent_border.insert(p);
        while (it != borders.end())
        {
            if (p.isNeighbour(*it))
            {
                adjacent_border.insert(it->begin(), it->end());
                borders.erase(it++);
            }
            else ++it;
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
    BorderList::const_iterator adjacent_border = find_if(borders.begin(), borders.end(), boost::bind( boost::mem_fn(&PointID::isNeighbour<PointSet>), p, _1));
    return (adjacent_border != borders.end());
}


bool MedianPoint::isBorderAdjacent(MedianPoint const& p) const
{
    if (!bbox.isAdjacent(p.bbox))
        return false;

    for (BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
    {
        PointSet::const_iterator point;
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

Point<float> MedianPoint::direction() const
{
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
    typedef std::list< PointSet > BorderList;
    for (BorderList::const_iterator it = p.borders.begin(); it != p.borders.end(); ++it)
    {
        io << " (";
        for (PointSet::const_iterator p = it->begin(); p != it->end(); ++p)
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

    if (median_set.count(PointID(x, y)))
        io << 1;
    else if (border_set.count(PointID(x, y)))
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
        MedianPoint::BorderList const& borders = it->second.borders;
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
        io << "    " << it->first << " " << it->second;
    }
    io << endl;
}

void Corridor::add(PointID const& p, MedianPoint const& descriptor)
{ return add(make_pair(p, descriptor)); }

void Corridor::add(std::pair<PointID, MedianPoint> const& p)
{
    median.insert(p);
    median_bbox.update(p.first);
    mergeBorders(p.second);
}
void Corridor::merge(Corridor const& corridor)
{
    // Force the overload selection on Corridor::add
    void (Corridor::*add)(pair<PointID, MedianPoint> const&) = &Corridor::add;
    for_each(corridor.median.begin(), corridor.median.end(), bind(mem_fn(add), this, _1));
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

PointID Corridor::adjacentEndpoint(PointID const& p) const
{
    for (MedianLine::const_iterator it = median.begin(); it != median.end(); ++it)
    {
        if (it->first.isNeighbour(p))
            return it->first;
    }
    return PointID();
}

bool Corridor::isNeighbour(PointID const& p) const
{
    if (!median_bbox.isNeighbour(p))
        return false;

    for (MedianLine::const_iterator it = median.begin(); it != median.end(); ++it)
    {
        if (it->first.isNeighbour(p))
            return true;
    }
    return false;
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

