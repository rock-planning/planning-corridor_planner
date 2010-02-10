#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include "point.hh"
#include "voronoi.hh"
#include "plan.hh"
#include "dstar.hh"
#include <vector>

namespace nav
{
    struct Endpoint
    {
        PointID point;
        int     corridor_idx;
        bool    side; // false for front() and true for back()

        Endpoint(PointID p, int idx, bool side)
            : point(p), corridor_idx(idx), side(side) {}
    };

    typedef std::map<PointID, std::list<VoronoiPoint>::const_iterator> VoronoiMap;
    typedef std::multimap< PointID, std::list<VoronoiPoint> > BranchMap;
    typedef std::list< std::list<Endpoint> > ConnectionPoints;
    typedef std::list< std::pair<ConnectionPoints::iterator, std::set<int> > > SplitPoints;

    struct CorridorExtractionState
    {
        Plan plan;
        GridGraph  graph;

        BranchMap  branches;
        VoronoiMap voronoiMap;

        ConnectionPoints connection_points;
        SplitPoints split_points;

        // Keeps whether a given corridor has connection points at both sides
        // (true) or only at one side (false)
        std::vector<int> simple_connectivity_corridors;

        int depth;

        CorridorExtractionState(int width, int height)
            : graph(width, height)
            , depth(0) {}

        VoronoiPoint const& front()
        {
            VoronoiMap::iterator it = voronoiMap.begin();
            return *(it->second);
        }

        void addBranch(PointID const& p, std::list<VoronoiPoint>& line);
    };


    /** This class takes a partitioning of the plan as an inside part and a
     * border, and turns it into either a Voronoi skeleton
     * (SkeletonExtraction.process()) or a corridor plan
     * (SkeletonExtraction.buildPlan()).
     *
     * It can accept a navigation function as input (processDStar), in which
     * case it will compute the inside/border based on an acceptable cost margin
     * (expand).
     */
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

        typedef std::list<VoronoiPoint>::const_iterator voronoi_const_iterator;

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

        /** Takes the height map built by one of the initialize* methods, and
         * extracts the voronoi diagrams out of it
         */
        std::list<VoronoiPoint> process();

        /** Takes the voronoi skeleton extracted by process() and turns it into
         * a set of branches. A branch is a line that has one start point and
         * one end point
         *
         * This is the first step to turn a voronoi skeleton into a corridor
         * plan
         */
        void extractBranches(std::list<VoronoiPoint> const& points,
                CorridorExtractionState& state);

        /** Takes the branche sets that are stored in the state and computes
         * the set of connection and split points
         */
        void computeConnections(CorridorExtractionState& state);

        /** Removes corridors that are dead ends (they neither connect two other
         * corridors with each other, nor do they lead to the start and end
         * points)
         */
        void removeDeadEnds(CorridorExtractionState& state, std::set<int> const& keepalive);

        /** \overload
         */
        std::pair<int, int> removeDeadEnds(CorridorExtractionState& state);

        void buildPlan(Plan& result, std::list<VoronoiPoint> const& points);

        std::vector<height_t> getHeightmap() const { return heightmap; }
    };
}

#endif

