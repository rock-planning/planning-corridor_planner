#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include "point.hh"
#include "voronoi.hh"
#include "plan.hh"
#include "dstar.hh"
#include <vector>

namespace nav
{
    class SkeletonExtraction
    {
    public:
        typedef int16_t height_t;
        static const height_t MAX_DIST = 999;

        int width, height;

        std::vector<height_t> heightmap;
        typedef std::map<height_t*, VoronoiPoint > ParentMap;
        ParentMap parents;
        PointSet border;

        PointID pointFromPtr(height_t const* ptr) const;
        
        typedef std::map<PointID, std::map<PointID, int> > ConnectionMap;

        /** Registers the connection described by \c connection_point between
         * the corridors listed in \c corridors and the corridor which index is
         * \c idx. In practice, it adds connections between
         *
         *   [idx, connection_point->first] and all [corridor, point] pairs in
         *     connection_point->second.
         */
        void registerConnections(PointID target_point, int target_idx,
                std::map<PointID, int> const& source_pairs,
                std::vector<Corridor>& corridors);

        void registerConnections(int idx, ConnectionMap::const_iterator endpoints, std::vector<Corridor>& corridors)
        { registerConnections(endpoints->first, idx, endpoints->second, corridors); }

    public:
        SkeletonExtraction(size_t width, size_t height);
        SkeletonExtraction(DStar const& nav_function, int x, int y, float expand);
        ~SkeletonExtraction();

        void displayHeightMap(std::ostream& io) const;

        /** Initializes the skeleton extraction process by extracting the
         * navigation zone from the provided navigation function (a D* result).
         * The resulting zone is the set of points P whose total cost 
         * (cost(P0 -> P) + cost(P -> P1)) is below expand * optimal_cost.
         */
        void initializeFromDStar(DStar const& nav_function, int x, int y, float expand);

        /** This is a convenience function which extracts the skeleton from a
         * dstar result. It is simply calling initializeFromDStar and process
         */
        std::list<VoronoiPoint> processDStar(DStar const& nav_function, int x, int y, float expand);

        /** Initializes the skeleton extraction process from a set of inside
         * points and a set of border points.
         */
        void initializeFromPointSets(PointSet const& inside, PointSet const& border);

        /** This is a convenience function which extracts the skeleton from two
         * sets of points. It is simply calling initializeFromPointSets and process
         */
        std::list<VoronoiPoint> processPointSets(PointSet const& inside, PointSet const& border);

        /** Returns true if the given point is inside the navigation zone (the
         * zone for which a skeleton needs to be built and false otherwise
         */
        bool isInside(int x, int y) const;

        /** Returns the set of points that are inside the considered region, and
         * the set of points that are border of it.
         *
         * This is for debugging purposes.
         */
        std::pair<PointSet, PointSet> getBorderAndInside() const;


        std::list<VoronoiPoint> process();

        /** Creates a graph out of a set of median points */
        void buildPlan(Plan& result, std::list<VoronoiPoint> const& points);
        void buildPixelMap(Plan& result) const;

        std::vector<height_t> getHeightmap() const { return heightmap; }
    };
}

#endif

