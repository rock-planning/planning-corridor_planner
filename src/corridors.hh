#ifndef CORRIDOR_PLANNER_CORRIDORS_HH
#define CORRIDOR_PLANNER_CORRIDORS_HH

#include <base/geometry/spline.h>

namespace corridors
{
    struct Corridor
    {
        double min_width, max_width;
        base::geometry::Spline<1> width_curve;
        base::geometry::Spline<3> median_curve;
        base::geometry::Spline<3> boundary_curves[2];
    };

    enum CORRIDOR_SIDE
    { FRONT_SIDE, BACK_SIDE };

    struct CorridorConnection
    {
        int from_idx;
        CORRIDOR_SIDE from_side;
        int to_idx;
        CORRIDOR_SIDE to_side;
    };

    struct Plan
    {
        int start_corridor, end_corridor;
        std::vector<Corridor> corridors;
        std::vector<CorridorConnection> connections;
    };
}

#endif

