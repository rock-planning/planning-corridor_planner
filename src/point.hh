#ifndef __NAV_POINT_HH
#define __NAV_POINT_HH

#include <iostream>
#include <set>
#include <vector>
#include <cmath>
#include <stdlib.h>

#include <Eigen/Core>

namespace corridor_planner {
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
        Point<T> operator += (Point<Other> p)
        { return *this = *this + p; }
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
            return std::sqrt(distance2(Point<T>(0, 0)));
        }

        template<typename Other>
        float distance2(Point<Other> const& p) const
        {
            return static_cast<float>(x - p.x) * static_cast<float>(x - p.x) + 
                 static_cast<float>(y - p.y) * static_cast<float>(y - p.y);
        }
        template<typename Other>
        float distance(Point<Other> const& p) const
        {
            return std::sqrt(distance2(p));
        }
    };


    struct PointID : public Point<int> { 
        PointID() : Point<int>() {}
        PointID(int x, int y)
            : Point<int>(x, y) {}
        PointID(Point<int> const& p)
            : Point<int>(p) {}

        Eigen::Vector3d toEigen() const
        { return Eigen::Vector3d(x, y, 0); }

        bool operator == (PointID const& p) const
        {
            return x == p.x && y == p.y;
        }

        bool isNeighbour(PointID const& p) const
        {
            return abs(x - p.x) <= 1 && abs(y - p.y) <= 1;
        }

        float distanceTo2(PointID const& other) const
        {
            return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
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
    typedef std::set<PointID>    PointSet;
    typedef std::vector<PointID> PointVector;

    template<typename T>
    std::ostream& operator << (std::ostream& io, Point<T> const& p)
    {
        io << "{ " << p.x << ", " << p.y << " }";
        return io;
    }
    template<typename Collection>
    void outputPointCollection(std::ostream& io, Collection const& set)
    {
        if (set.empty())
            io << "{ }";
        else
        {
            typename Collection::const_iterator it = set.begin();
            io << "{ " << *it;
            for (++it; it != set.end(); ++it)
                io << ", " << *it;
            io << " }";
        }
    }
    inline std::ostream& operator << (std::ostream& io, PointVector const& set)
    {
        outputPointCollection(io, set);
        return io;
    }
    inline std::ostream& operator << (std::ostream& io, PointSet const& set)
    {
        outputPointCollection(io, set);
        return io;
    }

    struct BoundingBox
    {
        bool initialized;
        PointID min;
        PointID max;

        BoundingBox()
            : initialized(false) {}

        void merge(BoundingBox const& box)
        {
            update(box.min);
            update(box.max);
        }

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

    inline PointID get_point(PointID const& p)
    { return p; }

    inline std::ostream& operator << (std::ostream& io, BoundingBox const& c)
    {
        io << "bbox[" << c.min << " => " << c.max << "]";
        return io;
    }

}

#endif
