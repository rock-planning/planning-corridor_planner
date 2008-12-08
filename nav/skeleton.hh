#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include <vector>
#include <stdint.h>
#include <queue>
#include <map>
#include <iosfwd>
#include "point.hh"

namespace Nav
{
    struct MedianPoint
    {
        std::set<PointID> border_points;
        int distance;
        MedianPoint()
            : distance(0)  {}
    };
    typedef std::map<PointID, MedianPoint> MedianLine;

    void displayMedianLine(std::ostream& io, MedianLine const& skel, int w, int h);

    struct Corridor
    {
        PointID    entry_points[4];
        MedianLine median;
        std::vector<int> connectivity;

        Corridor() {}
        Corridor(PointID const& p, MedianPoint const& info);
    };

    struct Skeleton : public std::vector<Corridor>
    {
        static Skeleton fromMedianLines(MedianLine const& points);
    };

    class SkeletonExtraction
    {
    public:
        int width, height;

        typedef int16_t height_t;
        std::vector<height_t> heightmap;

        void initializeHeightMap(PointSet const& inside);
        MedianLine propagateHeightMap(PointSet const& border);
        PointID pointFromPtr(height_t const* ptr) const;
        
    public:
        SkeletonExtraction(size_t width, size_t height);
        ~SkeletonExtraction();

        void displayHeightMap(std::ostream& io) const;

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        MedianLine processEdgeSet(PointSet const& edges, PointSet const& inside);

        std::vector<height_t> getHeightmap() const { return heightmap; }
    };
}

#endif

