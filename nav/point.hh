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

    struct BoundingBox
    {
        PointID min;
        PointID max;

        void update(PointID const& p)
        {
            min.x = std::min(p.x, min.x);
            min.y = std::min(p.y, min.y);
            max.x = std::max(p.x, max.x);
            max.y = std::max(p.y, max.y);
        }
        bool isIncluded(PointID const& p) const
        {
            return (p.x >= min.x) && (p.x <= max.x) &&
                (p.y >= min.y) && (p.y <= max.y);
        }
        bool isNeighbour(PointID const& p) const
        {
            return (p.x >= min.x - 1) && (p.x <= max.x + 1) &&
                (p.y >= min.y - 1) && (p.y <= max.y + 1);
        }

        bool isAdjacent(BoundingBox const& other) const
        {
            if (max.x < other.min.x - 1) return false;
            if (max.y < other.min.y - 1) return false;
            if (min.x > other.max.x + 1) return false;
            if (min.y > other.max.y + 1) return false;
            return true;
        }

        bool intersects(BoundingBox const& other) const
        {
            if (max.x < other.min.x) return false;
            if (max.y < other.min.y) return false;
            if (min.x > other.max.x) return false;
            if (min.y > other.max.y) return false;
            return true;
        }
    };
}

#endif
