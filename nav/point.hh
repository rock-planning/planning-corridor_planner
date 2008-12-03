#ifndef __NAV_POINT_HH
#define __NAV_POINT_HH

#include <iosfwd>
#include <set>

namespace Nav {
    struct PointID { 
        int x; 
        int y; 

        PointID() {}
        PointID(int x, int y)
            : x(x), y(y) {}
        bool operator == (PointID const& other) const
        { return x == other.x && y == other.y; }
        bool operator != (PointID const& other) const
        { return !(*this == other); }
        bool operator < (PointID const& other) const
        { return x < other.x || (x == other.x && y < other.y); }
    };
    typedef std::set<PointID> PointSet;

    std::ostream& operator << (std::ostream& io, PointID const& p);
}

#endif
