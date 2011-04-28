#ifndef NAV_CORRIDOR_PLANNER_HH
#define NAV_CORRIDOR_PLANNER_HH

#include <string>
#include <Eigen/Core>
#include <nav_graph_search/dstar.hpp>
#include <corridor_planner/voronoi.hh>
#include <corridor_planner/plan.hh>
#include <corridor_planner/corridors.hh>

#include <boost/noncopyable.hpp>

namespace envire
{
    class Environment;
}

namespace corridor_planner
{
    typedef nav_graph_search::TerrainClasses TerrainClasses;
    typedef nav_graph_search::TraversabilityMap TraversabilityMap;
    typedef nav_graph_search::DStar DStar;
        
    class SkeletonExtraction;
    class Plan;

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
        int strong_edge_map;
        std::string strong_edge_band;
        double strong_edge_threshold;

        // If this value is true, the NarrowWideAnnotation filter will be
        // passed on the resulting corridors
        bool narrow_wide_enable;
        double narrow_wide_narrow_threshold;
        double narrow_wide_wide_threshold;

    public:
        CorridorPlanner();
        ~CorridorPlanner();

        /** Load the terrain classes and traversability map */
        void init(std::string const& terrain_classes, std::string const& map, float min_width = 0);

        /** Call to set the start and goal positions */
        void setWorldPositions(Eigen::Vector2d const& current, Eigen::Vector2d const& goal);

        /** Call to set the start and goal positions */
        void setRasterPositions(Eigen::Vector2i const& current, Eigen::Vector2i const& goal);

        void setMarginFactor(double factor);

        /** Annotates the resulting corridors using the StrongEdgeAnnotation
         * filter
         */
        void enableStrongEdgeFilter(std::string const& env_path, int map_id, std::string const& band_name, double threshold);

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

