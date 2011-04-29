#ifndef NAV_PLAN_HH
#define NAV_PLAN_HH

#include <corridor_planner/voronoi.hh>
#include <nav_graph_search/dstar.hpp>

namespace corridor_planner
{
    class MergeResult;
    class Plan
    {
    protected:
        /** The start point for this plan */
        PointID m_start;
        /** The end point for this plan */
        PointID m_end;

        /** The navigation function on which this plan is defined */
        GridGraph m_nav_function;
        /** An index used to have unique names for the corridors */
        int m_corridor_names;

        typedef std::map< std::pair<int, int>, int > ConnectionTypes; 
	typedef std::map< Corridor::ConnectionDescriptor const*, int > EndpointTypes;

	std::vector<bool>  reach_flag;
	std::vector<float> reach_min_cost;
        std::vector<int>   orientations;

	bool markDirections_DFS(std::set< boost::tuple<int, bool, int, bool> >& result,
		std::vector<int>& stack, int in_side, int idx, int end_idx,
		float accumulated_cost_overhead, float cost_margin);
	void markDirections_cost();
        void removeBackToBackConnections(double margin_factor);

	void checkConsistency() const;

        /** Remove crossroads that simply connects two corridors (i.e. not real
         * crossroads)
         */
        void mergeSimpleCrossroads_directed();

    public:
        typedef std::vector< std::pair<int, bool> > Path;

        Plan();
        Plan(PointID start, PointID end, GridGraph const& nav_function = GridGraph());

        int findCorridorOf(PointID const& endp) const;
        PointID getStartPoint() const;
        PointID getEndPoint() const;
        GridGraph const& getNavigationFunction() const;
        void setStartPoint(PointID const& p);
        void setEndPoint(PointID const& p);
        void setNavigationFunction(GridGraph const& nav_function);

        void displayConnections(bool include_corridor_inner_connections = false) const;

        typedef std::vector<Corridor>::iterator corridor_iterator;

        /** The set of corridors in this plan, including the connections between
         * them */
        std::vector<Corridor> corridors;

        /** Create a new corridor and returns a reference to it
         *
         * Note that all reference to corridors will be invalidated as soon as a
         * mutating method is applied on the plan (simplify, newCorridor, ...)
         *
         * I.e. use the return value only to initialize it
         */
        Corridor& newCorridor();

        int width, height;
        /** This vector maps each pixel into its corresponding corridor */
        std::vector<uint8_t> pixel_map;

        void clear();
        void concat(Plan const& other);

        /** Removes the corridor from the set of corridors in this plan. It
         * updates all indexes in the other corridor connections, and removes
         * any connection that were existing towards the removed corridor
         */
        void removeCorridor(int idx);

        /** Remove the provided set of corridors, including the connections in
         * which these corridors are involved.
         *
         * Note that this changes the corridor indexes (but not their name).
         */
        void removeCorridors(std::set<int> const& corridors);
        bool removeCorridors(std::vector<bool> const& to_remove);

        /** Add a new connection in the plan */
        void addConnection(int from_idx, bool from_side, int to_idx, bool to_side);

        /** Removes all connections that have for source the given side of the
         * given corridor
         */
        void removeInboundConnectionsTo(int corridor_idx, bool side);

        /** Removes all connections that have for target the given side of the
         * given corridor
         */
        void removeOutboundConnectionsFrom(int corridor_idx, bool side);

        /** Move all connections that are defined on \c from_idx into the
         * corridor \c into_idx, updating the other end of the connection as
         * well.
         *
         * Of course, this method completely ignores the connections between
         * into_idx and from_idx
         */
        void moveConnections(size_t into_idx, size_t from_idx);

        /** Simplify the plan by removing useless corridors and merging
         * corridors that have a simple connections (i.e. remove crossroads that
         * link only two corridors).
         *
         * Useless corridors are the corridors that do not connect one endpoint
         * to another.
         *
         * @arg margin_factor { paths that leads to the goal with a cost less than
         *     margin_factor * optimal_cost are acceptable }
         * @arg min_width { the minimum acceptable width for a corridor. All
         *     corridors narrower than that will be removed. }
         */
        void simplify(double margin_factor, int min_width = 0, bool convert_to_dag = true);

        /** Split the given corridor at the specified place, and returns the new
         * corridor.
         *
         * After the split, the front part is saved in the original corridor
         * while the new corridor contains the back part.
         */
        Corridor& split(int corridor_idx, Corridor::voronoi_iterator it);

        /** Returns the index of the start corridor. It is always a singleton
         * corridor. Note that it must be created explicitely with
         * createEndpointCorridor
         */
        int findStartCorridor() const;

        /** Returns the index of the end corridor. It is always a singleton
         * corridor. Note that it must be created explicitely with
         * createEndpointCorridor
         */
        int findEndCorridor() const;

        /** Create a singleton corridor that contains +endpoint+, and connect it
         * to the nearest corridor that is already in the plan
         */
        void createEndpointCorridor(PointID const& endpoint, bool is_end);

        /** Inverts the geometrical direction of this corridor, and updates
         * the connections accordingly
         */
        void reverseCorridor(int corridor_idx);

        bool removeDeadEnds();
        bool removeDeadEnds(std::set<int> keepalive);

        void removeNullCorridors();
        void removeNullCorridors(std::set<int> keepalive);
        bool filterNullSingleton(int corridor_idx);

        /** Applies removeRedundantCorridorConnections(corridor_idx) on every
         * corridor
         */
        bool removeRedundantCorridorConnections();

        /** Filters out redundant connections that go out of the given corridor
         *
         * This function removes connections from a corridor's side when the
         * following pattern is found:
         *
         *   a:0 -> b:0
         *   a:0 -> b:1
         *
         * if this is found, then the outgoing connections of b:0 and b:1 are
         * copied to a:0, and the a:0->b:* connections are removed
         */
        bool removeRedundantCorridorConnections(int corridor_idx);

        void filterDeadEndPaths(std::vector<Path>& all_paths) const;
        void removeNarrowCorridors(std::vector<Path>& all_paths, double min_width, std::vector<bool>& to_delete);
        void computeAllPaths(std::vector<Path>& all_paths, Path& current_path, int next_path, int in_side, std::vector<int>& stack) const;

        bool removePointTurnConnections();
        void fixBoundaryOrdering();

        bool needsBoundarySwap(Corridor& source, bool source_side, Corridor& target, bool target_side) const;

        /** Computes all possible paths from the corridor plan, including those
         * that do not lead to the end corridor
         */
        void computeAllPaths(std::vector<Path>& all_paths) const;
        /** Removes the paths in +all_paths+ that contain corridors that are
         * marked as being deleted in +to_delete+
         */
        void filterDeletedPaths(std::vector<Path>& all_paths, std::vector<bool> to_delete) const;

    };
    std::ostream& operator << (std::ostream& io, Plan const& plan);
}

#endif

