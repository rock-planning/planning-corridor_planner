#ifndef NAV_PLAN_HH
#define NAV_PLAN_HH

#include "voronoi.hh"

namespace Nav
{
    class MergeResult;
    class Plan
    {
        /** Removes the useless corridors. Useless corridors are the corridors
         * that don't connect one endpoint with another
         *
         * @see simplify
         */
        void removeUselessCorridors(PointSet endpoints);

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
        int markNextCorridors(std::vector<int>& stack, int corridor_idx, std::vector<int>& useful) const;

        /** Remove crossroads that simply connects two corridors (i.e. not real
         * crossroads)
         */
        void mergeSimpleCrossroads();

    public:
        /** The set of corridors in this plan, including the connections between
         * them */
        std::vector<Corridor> corridors;
        /** The pixel-to-corridor map. It says, for each pixel, which corridor
         * own it
         */
        typedef std::vector<Corridor>::iterator corridor_iterator;

        int width, height;
        /** This vector maps each pixel into its corresponding corridor */
        std::vector<uint8_t> pixel_map;

        void clear();
        void concat(Plan const& other);

        /** Removes the corridor from the set of corridors in this plan. In
         * particular, in updates the corridor connections.
         */
        void removeCorridor(int idx);

        /** Move all connections that are defined on \c from_idx into the
         * corridor \c into_idx, updating the other end of the connection as
         * well.
         *
         * Of course, this method completely ignores the connections between
         * into_idx and from_idx
         */
        void moveConnections(int into_idx, int from_idx);

        void addAdjacentBorders(MedianPoint const& p0, MedianPoint const& p1, std::set<PointID>& result) const;
        void findAdjacentBorders(PointID p, Corridor const& corridor, std::set<PointID>& seen, std::set<PointID>& result) const;
        void buildCrossroads() const;

        /** Simplify the plan by removing useless corridors and merging
         * corridors that have a simple connections (i.e. remove crossroads that
         * link only two corridors).
         *
         * Useless corridors are the corridors that do not connect one endpoint
         * to another.
         *
         * @arg endpoints the set of endpoints
         */
        void simplify(PointSet endpoints);
    };
    std::ostream& operator << (std::ostream& io, Plan const& plan);
}

#endif

