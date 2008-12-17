#ifndef __NAV_POINT_HH
#define __NAV_POINT_HH

#include <iosfwd>
#include <set>
#include <cmath>
#include <stdlib.h>

namespace Nav {
    template<typename T>
    struct Point
    {
        T x, y;

        Point() : x(0), y(0) {}
        Point(T x, T y) : x(x), y(y) {}

        template<typename Other>
        Point(Point<Other> const& p)
            : x(p.x), y(p.y) {}

        bool operator == (Point<T> const& other) const
        { return x == other.x && y == other.y; }
        bool operator != (Point<T> const& other) const
        { return !(*this == other); }
        bool operator < (Point<T> const& other) const
        { return x < other.x || (x == other.x && y < other.y); }

        template<typename Scalar>
        Point<T> operator / (Scalar v) const
        { return Point<T>(x / v, y / v); }

        template<typename Other>
        Point<T> operator + (Point<Other> p) const
        { return Point<T>(x + p.x, y + p.y); }

        template<typename Other>
        Point<T> operator - (Point<Other> p) const
        { return Point<T>(x - p.x, y - p.y); }

        T operator * (Point<T> const& p) const
        { return x * p.x + y * p.y; }

        Point<float> normalize() const
        { 
            float d = norm();
            return Point<float>(x, y) / d;
        }

        float norm() const
        {
            return std::sqrt(static_cast<float>(x * x) + static_cast<float>(y * y));
        }

        template<typename Other>
        float distance2(Point<Other> const& p) const
        {
            return static_cast<float>(x * p.x) + static_cast<float>(y * p.y);
        }
    };


    struct PointID : public Point<int> { 
        PointID() : Point<int>() {}
        PointID(int x, int y)
            : Point<int>(x, y) {}
        PointID(Point<int> const& p)
            : Point<int>(p) {}

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
        bool initialized;
        PointID min;
        PointID max;

        BoundingBox()
            : initialized(false) {}

        void update(PointID const& p)
        {
            if (!initialized)
            {
                min = max = p;
                initialized = true;
            }
            else
            {
                min.x = std::min(p.x, min.x);
                min.y = std::min(p.y, min.y);
                max.x = std::max(p.x, max.x);
                max.y = std::max(p.y, max.y);
            }
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
