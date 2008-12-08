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
    struct SkeletonInfo
    {
        std::set<PointID> parents;
        SkeletonInfo(PointID p)
        {
            parents.insert(p);
        }
    };
    typedef std::map<PointID, SkeletonInfo> Skeleton;

    void displaySkeleton(std::ostream& io, Skeleton const& skel, int w, int h);

    class SkeletonExtraction
    {
    public:
        int width, height;

        typedef int16_t height_t;
        std::vector<height_t> heightmap;

        void initializeHeightMap(PointSet const& inside);
        std::multimap<PointID, PointID> propagateHeightMap(PointSet const& border);
        PointID pointFromPtr(height_t const* ptr) const;
        
    public:
        SkeletonExtraction(size_t width, size_t height);
        ~SkeletonExtraction();

        void displayHeightMap(std::ostream& io) const;

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        Skeleton processEdgeSet(PointSet const& edges, PointSet const& inside);

        std::vector<height_t> getHeightmap() const { return heightmap; }
    };
}

#endif

