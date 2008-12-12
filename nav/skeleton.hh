#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include "point.hh"
#include "voronoi.hh"
#include "plan.hh"
#include <vector>

namespace Nav
{
    class SkeletonExtraction
    {
    public:
        int width, height;

        typedef int16_t height_t;
        std::vector<height_t> heightmap;
        typedef std::map<height_t*, MedianPoint > ParentMap;
        ParentMap parents;

        void initializeHeightMap(PointSet const& inside);
        MedianLine propagateHeightMap(PointSet const& border);
        PointID pointFromPtr(height_t const* ptr) const;
        
        typedef std::map<PointID, std::map<PointID, int> > ConnectionMap;
        void registerConnections(int idx, std::vector<Corridor>& corridors, ConnectionMap::const_iterator connection_point);

    public:
        SkeletonExtraction(size_t width, size_t height);
        ~SkeletonExtraction();

        void displayHeightMap(std::ostream& io) const;

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        MedianLine processEdgeSet(PointSet const& edges, PointSet const& inside);

        /** Creates a graph out of a set of median points */
        Plan buildPlan(MedianLine points);
        void buildPixelMap(Plan& result) const;

        std::vector<height_t> getHeightmap() const { return heightmap; }
    };
}

#endif

