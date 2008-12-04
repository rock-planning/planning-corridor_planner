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
        std::vector<uint8_t>     heightmap;

        struct CandidateComparison
        {
            bool operator ()(uint8_t const* a, uint8_t const* b) const
            { return *a > *b; }
        };
        typedef std::priority_queue<uint8_t*, std::vector<uint8_t*>, CandidateComparison> CandidateSet;

        void initializeHeightMap(size_t max, PointSet const& inside);
        std::multimap<PointID, PointID> propagateHeightMap(PointSet const& border);
        PointID pointFromPtr(uint8_t* ptr) const;
        
    public:
        SkeletonExtraction(size_t width, size_t height);
        ~SkeletonExtraction();

        void displayHeightMap(std::ostream& io) const;

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        Skeleton processEdgeSet(PointSet const& edges, PointSet const& inside);

        std::vector<uint8_t> getHeightmap() const { return heightmap; }
    };
}

#endif

