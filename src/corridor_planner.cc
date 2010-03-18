#include "corridor_planner.hh"

#include "skeleton.hh"
#include "voronoi.hh"
#include "dstar.hh"
#include "plan.hh"

#include <stdexcept>


using namespace nav;

CorridorPlanner::CorridorPlanner()
    : map(0), skeleton(0), dstar(0)
    , m_state(DSTAR), m_expand(1.1)
{
}

CorridorPlanner::~CorridorPlanner()
{
    delete map;
    delete skeleton;
    delete dstar;
}

void CorridorPlanner::requireProcessing(CorridorPlanner::STATES state)
{ m_state = std::min(m_state, static_cast<int>(state)); }
bool CorridorPlanner::isProcessingRequired(CorridorPlanner::STATES state) const
{ return m_state <= state; }
void CorridorPlanner::processed()
{ m_state++; }

/** Load the terrain classes and traversability map */
void CorridorPlanner::init(std::string const& terrain_classes, std::string const& map_file)
{
    classes = TerrainClass::load(terrain_classes);
    delete map;
    map     = TraversabilityMap::load(map_file, classes);
    if (!map)
        throw std::runtime_error("cannot load the specified map");

    delete dstar;
    dstar   = new DStar(*map, classes);
    delete skeleton;
    skeleton = new SkeletonExtraction(map->xSize(), map->ySize());

}

void CorridorPlanner::setMarginFactor(double factor)
{
    m_expand = factor;
    requireProcessing(SKELETON);
}

/** Call to set the start and goal positions */
void CorridorPlanner::setWorldPositions(Eigen::Vector2d const& current, Eigen::Vector2d const& goal)
{
    PointID current_i = map->toLocal(Eigen::Vector3d(current.x(), current.y(), 0)),
        goal_i = map->toLocal(Eigen::Vector3d(goal.x(), goal.y(), 0));
    std::cerr << "current position:\n"
        << "   world: " << current
        << "   raster: " << Eigen::Vector2i(current_i.x, current_i.y) << std::endl;
    std::cerr << "goal position:\n"
        << "   world: " << goal
        << "   raster: " << Eigen::Vector2i(goal_i.x, goal_i.y) << std::endl;

    setRasterPositions(Eigen::Vector2i(current_i.x, current_i.y), Eigen::Vector2i(goal_i.x, goal_i.y));
}

/** Call to set the start and goal positions */
void CorridorPlanner::setRasterPositions(Eigen::Vector2i const& current, Eigen::Vector2i const& goal)
{
    if (current != m_current)
    {
        m_current = current;
        requireProcessing(SKELETON);
    }
    if (goal != m_goal)
    {
        m_goal = goal;
        dstar->initialize(goal.x(), goal.y());
        requireProcessing(DSTAR);
    }
}

/** Call to notify the planner that the map changed at the given
 * position */
void CorridorPlanner::updatedMap(int x, int y)
{
    dstar->updated(x, y);
    requireProcessing(DSTAR);
}

/** Run the D* pass */
void CorridorPlanner::computeDStar()
{
    if (isProcessingRequired(DSTAR))
    {
        dstar->update();
        processed();
    }
}

/** Run the skeleton extraction pass */
void CorridorPlanner::extractSkeleton()
{
    if (isProcessingRequired(SKELETON))
    {
        voronoi_points =
                skeleton->processDStar(*dstar, m_current.x(), m_current.y(), m_expand);
        processed();
    }
}

/** Run the plan computation pass */
void CorridorPlanner::computePlan()
{
    if (isProcessingRequired(PLAN))
    {
        plan = skeleton->buildPlan(PointID(m_current.x(), m_current.y()), PointID(m_goal.x(), m_goal.y()), dstar->graph(), voronoi_points);
        processed();
    }
}

/** Run the plan simplification pass. The resulting plan is returned by
 * getPlan()
 */
void CorridorPlanner::simplifyPlan()
{
    if (isProcessingRequired(PLAN_SIMPLIFICATION))
    {
        plan.simplify();
        processed();
    }
}

/** Do all the passes in the right order */
Plan const& CorridorPlanner::compute()
{
    computeDStar();
    extractSkeleton();
    computePlan();
    simplifyPlan();
    return plan;
}

