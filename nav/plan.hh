#ifndef NAV_PLAN_HH
#define NAV_PLAN_HH

#include "voronoi.hh"

namespace Nav
{
    class MergeResult;
    struct Plan
    {
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

        void addAdjacentBorders(MedianPoint const& p0, MedianPoint const& p1, std::set<PointID>& result) const;
        void findAdjacentBorders(PointID p, Corridor const& corridor, std::set<PointID>& seen, std::set<PointID>& result) const;
        void buildCrossroads() const;
    };
    std::ostream& operator << (std::ostream& io, Plan const& plan);
}

#endif

