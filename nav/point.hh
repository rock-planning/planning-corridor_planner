#ifndef __NAV_POINT_HH
#define __NAV_POINT_HH

#include <iosfwd>
#include <set>
#include <stdlib.h>

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

        bool isNeighbour(PointID const& p) const
        {
            return abs(x - p.x) <= 1 && abs(y - p.y) <= 1;
        }

        template<typename Container>
        bool isNeighbour(Container const& container) const
        {
            typename Container::const_iterator it,
                     end = container.end();
            for (it = container.begin(); it != end; ++it)
            {
                if (it->isNeighbour(*this))
                    return true;
            }
            return false;
        }
    };
    typedef std::set<PointID> PointSet;

    std::ostream& operator << (std::ostream& io, PointID const& p);
}

#endif
