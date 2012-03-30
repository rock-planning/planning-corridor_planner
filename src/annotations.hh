#ifndef CORRIDOR_PLANNER_ANNOTATIONS_HH
#define CORRIDOR_PLANNER_ANNOTATIONS_HH

#include <corridor_planner/corridors.hh>
#include <corridor_planner/plan.hh>
#include <envire/maps/Grid.hpp>
#include <string>

namespace envire {
    class MLSGrid;
}

namespace corridor_planner {
    /** Base class for annotation filters
     *
     * Annotation filters are used to annotate the corridor boundaries and/or
     * median curve with certain additional information.
     *
     * For instance, the StrongEdgeAnnotation filter will mark with STRONG_EDGE
     * the parts of the boundaries that for which the local slope is higher than
     * a certain threshold, allowing to represent the parts of the corridors for
     * which the corridor boundaries are physical and the ones for which it is
     * more of a virtual boundary
     */
    struct AnnotationFilter {
        std::string const symbol;

        AnnotationFilter(std::string const& symbol);

        /** Requires this filter to annotate the given corridor
         *
         * @arg index the index that should be used in Corridor::annotations
         * @arg corridor the corridor that should be annotated
         *
         * Note that Corridor::annotations has already been resized so that
         * a \c index slot exists
         */
        virtual void annotate(int index, corridors::Corridor& corridor) = 0;

        /** Apply the given filter to the given plan */
        static void apply(AnnotationFilter& filter, corridors::Plan& plan);
    };

    /** Annotation filter that annotates the boundaries that have a "physical"
     * meaning with the STRONG_EDGE symbol
     *
     * This filter marks with a STRONG_EDGE symbol the parts of the boundaries
     * that for which the local maximum step (as stored in the map) is higher
     * than a certain threshold, allowing to represent the parts of the
     * corridors for which the corridor boundaries are physical and the ones for
     * which it is more of a virtual boundary
     *
     * In the annotated segments, the \c symbol field can take the following
     * values:
     *
     * <ul>
     * <li>UNKNOWN=0: no information for that segment
     * <li>STRONG=1: the local step size is above the given threshold
     * </ul>
     */
    struct StrongEdgeAnnotation : public AnnotationFilter {
        enum Symbols {
            UNKNOWN,
            STRONG
        };
        /** The map that contains the local step (in meters) */
        envire::Grid<double> const* map;
        /** The band that should be used in \c map */
        std::string band;
        /** The threshold above which a segment should be marked */
        double threshold;

        StrongEdgeAnnotation(envire::Grid<double> const* map, std::string const& band_name, double threshold);

        void annotateCurve(corridors::Corridor::Annotations& result, base::geometry::Spline<3> const& curve);

        void annotate(int index, corridors::Corridor& corridor);
    };

    /** Annotation filter that annotates the parts of a corridor in NARROW or
     * WIDE classes, if the corridor local width is resp. below a certain
     * threshold or above another threshold
     *
     * In the annotated segments, the \c symbol field can take the following
     * values:
     *
     * <ul>
     * <li>NARROW=0: the corridor is locally narrower than wide_threshold
     * <li>WIDE=1: the corridor is locally wider than wide_threshold
     * </ul>
     */
    struct NarrowWideAnnotation : public AnnotationFilter {
        enum Symbols {
            NARROW,
            WIDE
        };
        /** The threshold below which a segment is marked as NARROW */
        double narrow_threshold;
        /** The threshold above which a segment is marked as WIDE */
        double wide_threshold;

        NarrowWideAnnotation(double narrow_threshold, double wide_threshold);

        virtual void annotate(int index, corridors::Corridor& corridor);
    };

    /** Annotation filter that annotates the median line of a corridor as either
     * KNOWN or UNKNOWN. It currently looks only at the cells on the median
     * line. A better implementation would look at the ration between known and
     * unknown cells in a corridor segment.
     */
    struct KnownUnknownAnnotation : public AnnotationFilter {
        enum Symbols {
            KNOWN,
            UNKNOWN
        };

        /** The map that contains the local step (in meters) */
        envire::Grid<uint8_t> const* map;
        /** The band that should be used in \c map */
        std::string band_name;
        /** The value used in \c map to mark unknown */
        uint8_t unknown_class;

        KnownUnknownAnnotation(envire::Grid<uint8_t> const* map, std::string const& band_name,
                uint8_t unknown_class);

        virtual void annotate(int index, corridors::Corridor& corridor);
    };
}

#endif

