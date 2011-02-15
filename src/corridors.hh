#ifndef CORRIDOR_PLANNER_CORRIDORS_HH
#define CORRIDOR_PLANNER_CORRIDORS_HH

#include <base/geometry/spline.h>

namespace corridors
{
    /** Representation of a corridor
     *
     * A corridor is a geometric structure that has only one input and one
     * output
     */
    struct Corridor
    {
        /** The minimum and maximum width of this corridor */
        double min_width, max_width;

        /** The local width of the corridor
         *
         * This is actually the half-width, i.e. the distance between the local
         * median point and the closest border.
         *
         * It is parametrized as \c median_curve, it is used for instance:
         *
         *   t = median_curve.findClosestPoint(ref_point);
         *   w = width_curve.getPoint(t)(0, 0)
         */
        base::geometry::Spline<1> width_curve;

        /** The median curve, i.e. the center curve of the corridor */
        base::geometry::Spline<3> median_curve;
        /** The boundary curves. boundary_curves[0] is on the right of the
         * median w.r.t. the direction of travel, while boundary_curves[1] is on
         * the left
         *
         * The curves are oriented in the same direction than \c median_curve,
         * i.e. the starting points of each boundary and of the median form one
         * end zone of the corridor, and the end points the other.
         */
        base::geometry::Spline<3> boundary_curves[2];
    };

    enum CORRIDOR_SIDE
    { FRONT_SIDE, BACK_SIDE };

    /** Connection between two corridors
     *
     * Connections are oriented, i.e. they represent that one can travel \c from
     * a corridor \c to another corridor
     */
    struct CorridorConnection
    {
        /** The index of the corridor we are travelling from, as an index in the
         * \c corridors field of a Plan data structure
         */
        int from_idx;
        /** The side out of which we are getting
         *
         * FRONT_SIDE is the start points of the corridor curves, BACK_SIDE the
         * end points
         */
        CORRIDOR_SIDE from_side;
        /** The index of the corridor we are travelling into, as an index in the
         * \c corridors field of a Plan data structure
         */
        int to_idx;
        /** The side into which we are getting
         *
         * FRONT_SIDE is the start points of the corridor curves, BACK_SIDE the
         * end points
         */
        CORRIDOR_SIDE to_side;
    };

    /** A corridor plan
     *
     * A corridor plan is a set of corridors and the connections that represent
     * how to travel from a corridor into another
     */
    struct Plan
    {
        /** The indexes of the starting corridor and of the ending one, as
         * indexes in \c corridors
         */
        int start_corridor, end_corridor;
        /** The geometric representation of the corridors
         */
        std::vector<Corridor> corridors;
        /** The connections between the corridors
         */
        std::vector<CorridorConnection> connections;
    };
}

#endif

