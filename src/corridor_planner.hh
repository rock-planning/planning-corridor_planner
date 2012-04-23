#ifndef NAV_CORRIDOR_PLANNER_HH
#define NAV_CORRIDOR_PLANNER_HH

#include <string>
#include <Eigen/Core>
#include <nav_graph_search/dstar.hpp>
#include <corridor_planner/voronoi.hh>
#include <corridor_planner/plan.hh>
#include <corridor_planner/corridors.hh>
#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>

#include <boost/noncopyable.hpp>

namespace corridor_planner
{
    typedef nav_graph_search::TerrainClasses TerrainClasses;
    typedef nav_graph_search::TraversabilityMap TraversabilityMap;
    typedef nav_graph_search::DStar DStar;
        
    class SkeletonExtraction;
    class Plan;

    struct PlanningFailed : public std::runtime_error
    {
        PlanningFailed(std::string const& msg)
            : std::runtime_error(msg) {}
    };

    struct CostCutoffReached : public PlanningFailed
    {
        CostCutoffReached(std::string const& msg)
            : PlanningFailed(msg) {}
    };

    /** Simple interface to the corridor planner functionality
     */
    class CorridorPlanner : boost::noncopyable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TerrainClasses      classes;
        TraversabilityMap*  map;
        SkeletonExtraction* skeleton;
        DStar*              dstar_to_start;
        DStar*              dstar_to_goal;
        Plan                plan;
        corridors::Plan     final;

        /** The minimum allowed width of the corridors, in meters */
        float min_width;

        std::list<VoronoiPoint>  voronoi_points;

        envire::Environment* env;

    private:
        Eigen::Vector2i m_current;
        Eigen::Vector2i m_goal;

        enum STATES
        {
            DSTAR, SKELETON, PLAN, PLAN_SIMPLIFICATION, ANNOTATIONS, DONE
        };

        int m_state;
        void requireProcessing(STATES state);
        bool isProcessingRequired(STATES state) const;
        void processed();

        /** Updates the \c result attribute from the data in \c plan */
        void exportPlan();

        double m_expand;

        // If this value is true, the StrongEdgeAnnotation filter will be
        // passed on the resulting corridors
        bool strong_edge_enable;
        envire::Grid<float>::Ptr strong_edge_map;
        std::string strong_edge_band;
        double strong_edge_threshold;

        // If this value is true, the NarrowWideAnnotation filter will be
        // passed on the resulting corridors
        bool narrow_wide_enable;
        double narrow_wide_narrow_threshold;
        double narrow_wide_wide_threshold;

        // If this value is true, the KnownUnknownAnnotation filter will be
        // passed on the resulting corridors
        bool known_unknown_enable;
        envire::Grid<uint8_t>::Ptr known_unknown_map;
        std::string known_unknown_band;
        uint8_t known_unknown_class;

    public:
        CorridorPlanner();
        ~CorridorPlanner();

        /** Load the terrain classes and traversability map */
        void init(std::string const& terrain_classes, std::string const& map, float min_width = 0, float cost_cutoff = std::numeric_limits<float>::max());

        /** Load the terrain classes and traversability map */
        void init(std::string const& terrain_classes,
                envire::Grid<uint8_t> const& map, std::string const& band_name,
                float min_width = 0, float cost_cutoff = std::numeric_limits<float>::max());

        /** Load the terrain classes and traversability map */
        void init(nav_graph_search::TerrainClasses const& classes,
                nav_graph_search::TraversabilityMap* map,
                float min_width = 0, float cost_cutoff = std::numeric_limits<float>::max());

        /** Call to set the start and goal positions */
        void setWorldPositions(Eigen::Vector2d const& current, Eigen::Vector2d const& goal);

        /** Call to set the start and goal positions */
        void setRasterPositions(Eigen::Vector2i const& current, Eigen::Vector2i const& goal);

        void setMarginFactor(double factor);

        /** @overload
         */
        void enableStrongEdgeFilter(std::string const& env_path, std::string map_id, std::string const& band_name, double threshold);

        /** Annotates the resulting corridors using the StrongEdgeAnnotation
         * filter
         */
        void enableStrongEdgeFilter(envire::Grid<float>::Ptr step_size, std::string const& band_name, double threshold);

        /** Disables the strong edge filter as enabled by enableStrongEdgeFilter
         */
        void disableStrongEdgeFilter();

        /** Annotates the resulting corridors using the NarrowWideAnnotation
         * filter
         */
        void enableNarrowWideFilter(double narrow_threshold, double wide_threshold);

        /** Disables the strong edge filter as enabled by
         * enableNarrowWideAnnotation
         */
        void disableNarrowWideFilter();

        /** Annotates the resulting corridors using the KnownUnknownAnnotation
         * filter
         */
        void enableKnownUnknownFilter(envire::Grid<uint8_t>::Ptr grid, std::string const& band_name, uint8_t unknown_class);

        /** Disables the KnownUnknownAnnotation filter as enabled by
         * enableKnownUnknownFilter
         */
        void disableKnownUnknownFilter();

        /** Call to notify the planner that the map changed at the given
         * position */
        void updatedMap(int x, int y);
        
        /** Run the D* pass */
        void computeDStar();

        /** Run the skeleton extraction pass */
        void extractSkeleton();

        /** Run the plan computation pass */
        void computePlan();

        /** Run the plan simplification pass. The resulting plan is returned by
         * getPlan()
         */
        void simplifyPlan();

        /** Run the annotation pass on the generated corridors
        */
        void annotateCorridors();

        /** Called when the planning is finished
         */
        void done();

        /** Returns the planner's result. Only valid if all stages have been
         * called already
         *
         * See also compute()
         */
        corridors::Plan const& result() const;

        /** Do all the passes in the right order */
        corridors::Plan const& compute();
    };
}

#endif

