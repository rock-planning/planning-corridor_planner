#ifndef CORRIDOR_PLANNER_CORRIDORS_HH
#define CORRIDOR_PLANNER_CORRIDORS_HH

#include <base/geometry/Spline.hpp>

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

        enum CURVE_INDEX {
            WITH_CURVE = 0,
            BOUNDARY_CURVE_LEFT = 1,
            BOUNDARY_CURVE_RIGHT = 2
        };

        /** A segment of a curve that has been annotated
         *
         * The meanign of \c symbol is dependent on the annotation filter used
         */
        struct AnnotatedSegment {
            double start;
            double end;
            int symbol;

            AnnotatedSegment()
                : start(0), end(0), symbol(0) {}
            AnnotatedSegment(double start, double end, int symbol)
                : start(start), end(end), symbol(symbol) {}
        };

        /** A set of annotated curves
         *
         * The segments are supposed to be ordered in increasing order on the
         * curve
         */
        typedef std::vector<AnnotatedSegment> Annotations;

        /** The corridor annotations for the median curve (index = 0) and the
         * boundary curves (indexes = 1, 2)
         *
         * The annotations are symbolic values that are associated with portions
         * of the curves. The symbols are defined as indexes and the portions of
         * the curves as pairs of parameters [from, to]. The Plan object then
         * contains a mapping from symbol index to symbol name (as a string)
         */
        std::vector<Annotations> annotations[3];
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
        /** A mapping from annotation index to annotation symbol for the curve
         * annotations
         *
         * See Corridor::annotations for a more in-depth description
         */
        std::vector<std::string> annotation_symbols;
        /** The geometric representation of the corridors
         */
        std::vector<Corridor> corridors;
        /** The connections between the corridors
         */
        std::vector<CorridorConnection> connections;

        /** Returns the index in \c annotation_index that match the given symbol
         */
        int findAnnotationIndex(std::string const& name)
        {
            for (unsigned int i = 0; i < annotation_symbols.size(); ++i)
            {
                if (annotation_symbols[i] == name)
                    return i;
            }
            return -1;
        }
    };
}

#endif

