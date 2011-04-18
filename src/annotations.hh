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
     * <li>STRONG: the local step size is above the given threshold
     * <li>UNKNOWN: no information for that segment
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
}

#endif

