#ifndef NAV_CORRIDOR_PLANNER_HH
#define NAV_CORRIDOR_PLANNER_HH

#include <string>
#include <Eigen/Core>
#include "dstar.hh"
#include "voronoi.hh"
#include "plan.hh"
#include "corridors.hh"

#include <boost/noncopyable.hpp>

namespace corridor_planner
{
    class SkeletonExtraction;
    class TraversabilityMap;
    class Plan;

    /** Simple interface to the corridor planner functionality
     */
    class CorridorPlanner : boost::noncopyable
    {
    public:
        TerrainClasses      classes;
        TraversabilityMap*  map;
        SkeletonExtraction* skeleton;
        DStar*              dstar;
        Plan                plan;
        corridors::Plan     final;

        /** The minimum allowed width of the corridors, in meters */
        float min_width;

        std::list<VoronoiPoint>  voronoi_points;

    private:
        Eigen::Vector2i m_current;
        Eigen::Vector2i m_goal;

        enum STATES
        {
            DSTAR, SKELETON, PLAN, PLAN_SIMPLIFICATION, DONE
        };

        int m_state;
        void requireProcessing(STATES state);
        bool isProcessingRequired(STATES state) const;
        void processed();

        /** Updates the \c result attribute from the data in \c plan */
        void exportPlan();

        double m_expand;

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

