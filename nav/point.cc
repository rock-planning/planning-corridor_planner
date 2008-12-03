#include "point.hh"
#include <iostream>

std::ostream& Nav::operator << (std::ostream& io, PointID const& p)
{
    io << "{ " << p.x << ", " << p.y << " }";
    return io;
}

