#include "voronoi.hh"
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>

#include <iostream>

using namespace std;
using namespace Nav;

void MedianPoint::addBorderPoint(PointID const& p)
{
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
        for_each(border->begin(), border->end(), boost::bind( boost::mem_fn(&MedianPoint::addBorderPoint), this, _1));
}

bool MedianPoint::isAdjacent(MedianPoint const& p) const
{
    for (BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
    {
        PointSet::const_iterator point;
        for (point = border->begin(); point != border->end(); ++point)
        {
            BorderList::const_iterator adjacent_border = find_if(p.borders.begin(), p.borders.end(), boost::bind( boost::mem_fn(&PointID::isNeighbour<PointSet>), *point, _1));
            if (adjacent_border != p.borders.end())
                break;
        }
        if (point == border->end())
            return false;
    }
    return true;
}

void MedianPoint::displayBorders(ostream& io) const
{
    for (BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
    {
        io << " (";
        for (PointSet::const_iterator p = it->begin(); p != it->end(); ++p)
            io << " " << *p;
        io << ")";
    }
}

void Nav::displayMedianLine(ostream& io, MedianLine const& skel, int w, int h)
{
    cerr << "Bitmap:" << endl;
    io << "  ";
    for (int x = 0; x < w; ++x)
        io << " " << std::setw(2) << x;
    io << endl;
    for (int y = 0; y < h; ++y)
    {
        io << std::setw(2) << y;
        for (int x = 0; x < w; ++x)
        {
            if (skel.count(PointID(x, y)))
                io << " " << std::setw(2) << 1;
            else
                io << "  -";
        }
        io << endl;
    }

    cerr << "Parent list:" << std::endl;
    for (MedianLine::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        cerr << "  " << it->first << " ";
        it->second.displayBorders(cerr);
        cerr << endl;
    }
}

