#ifndef NAV_PLAN_HH
#define NAV_PLAN_HH

#include "voronoi.hh"

namespace Nav
{
    struct Plan
    {
        std::vector<Corridor> corridors;
        typedef std::vector<Corridor>::iterator corridor_iterator;

        int width, height;
        /** This vector maps each pixel into its corresponding corridor */
        std::vector<uint8_t> pixel_map;

        void removeCorridor(int idx);
    };
    std::ostream& operator << (std::ostream& io, Plan const& plan);
}

#endif

