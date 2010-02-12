#ifndef NAV_PLAN_HH
#define NAV_PLAN_HH

#include "voronoi.hh"
#include "dstar.hh"

namespace nav
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

        /** Marks in \c useful the useless corridors. Useless corridors are the
         * corridors that don't connect one endpoint with another
         *
         * @see simplify
         */
        void markUselessCorridors(std::vector<int>& useful);

        /** Removes the corridors that are marked as useless in \c useful.
         *
         * @see simplify
         */
        void removeUselessCorridors(std::vector<int>& useful);

        /** Constants used by markNextCorridors and removeUselessCorridors */
        static const int USEFUL = 1;
        static const int NOT_USEFUL = 2;

        /** This method is used by removeUselessCorridors to search for useless
         * corridors. We use for that a DFS, where
         * <ul>
         *   <li> \c stack is the DFS stack
         *   <li> \c corridor_idx is the corridor to be considered at this step
         *   <li> \c useful is the set of usefulness flags
         * </ul>
         *
         * The method calls itself recursively on the children of \c
         * corridor_idx whose usefulness is still undetermined. If one child is
         * useful, then the corridor is useful as well.
         */
        int markNextCorridors(std::set<int>& stack, int corridor_idx, std::vector<int>& useful) const;

        typedef std::map< std::pair<int, int>, int > ConnectionTypes; 
	typedef std::map< Corridor::ConnectionDescriptor const*, int > EndpointTypes;

	std::vector<bool>  reach_flag;
	std::vector<float> reach_min_cost;
        std::vector<int>   orientations;

	bool markDirections_DFS(std::set< boost::tuple<int, bool, int, bool> >& result,
		std::vector<int>& stack, int in_side, int idx, int end_idx,
		float accumulated_cost_overhead, float cost_margin);
	void markDirections_cost();
        void removeBackToBackConnections();

	void checkConsistency() const;

        /** Remove crossroads that simply connects two corridors (i.e. not real
         * crossroads)
         */
        void mergeSimpleCrossroads_directed();

    public:
        Plan();
        Plan(PointID start, PointID end, GridGraph const& nav_function = GridGraph());

        int findCorridorOf(PointID const& endp) const;
        PointID getStartPoint() const;
        PointID getEndPoint() const;
        GridGraph const& getNavigationFunction() const;
        void setStartPoint(PointID const& p);
        void setEndPoint(PointID const& p);
        void setNavigationFunction(GridGraph const& nav_function);

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
         * @arg endpoints the set of endpoints
         */
        void simplify();

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
    };
    std::ostream& operator << (std::ostream& io, Plan const& plan);
}

#endif

