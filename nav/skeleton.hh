#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include "point.hh"
#include "voronoi.hh"
#include <vector>

namespace Nav
{
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

