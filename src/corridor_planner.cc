#include "corridor_planner.hh"

#include "skeleton.hh"
#include "voronoi.hh"
#include "dstar.hh"
#include "plan.hh"

#include <stdexcept>


using namespace corridor_planner;

CorridorPlanner::CorridorPlanner()
    : map(0), skeleton(0), dstar_to_start(0), dstar_to_goal(0)
    , min_width(0)
    , m_state(DSTAR), m_expand(1.1)
{
}

CorridorPlanner::~CorridorPlanner()
{
    delete map;
    delete skeleton;
    delete dstar_to_start;
    delete dstar_to_goal;
}

void CorridorPlanner::requireProcessing(CorridorPlanner::STATES state)
{ m_state = std::min(m_state, static_cast<int>(state)); }
bool CorridorPlanner::isProcessingRequired(CorridorPlanner::STATES state) const
{ return m_state <= state; }
void CorridorPlanner::processed()
{ m_state++; }

/** Load the terrain classes and traversability map */
void CorridorPlanner::init(std::string const& terrain_classes, std::string const& map_file, float min_width)
{
    classes = TerrainClass::load(terrain_classes);
    delete map;
    map     = TraversabilityMap::load(map_file, classes);
    if (!map)
        throw std::runtime_error("cannot load the specified map");

    delete dstar_to_start;
    delete dstar_to_goal;
    dstar_to_start = new DStar(*map, classes);
    dstar_to_goal  = new DStar(*map, classes);
    delete skeleton;
    skeleton = new SkeletonExtraction(map->xSize(), map->ySize());

    this->min_width = min_width;

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
        dstar_to_start->initialize(current.x(), current.y());
        requireProcessing(DSTAR);
    }
    if (goal != m_goal)
    {
        m_goal = goal;
        dstar_to_goal->initialize(goal.x(), goal.y());
        requireProcessing(DSTAR);
    }
}

/** Call to notify the planner that the map changed at the given
 * position */
void CorridorPlanner::updatedMap(int x, int y)
{
    dstar_to_start->updated(x, y);
    dstar_to_goal->updated(x, y);
    requireProcessing(DSTAR);
}

/** Run the D* pass */
void CorridorPlanner::computeDStar()
{
    if (isProcessingRequired(DSTAR))
    {
        dstar_to_start->update();
        dstar_to_goal->update();
        processed();
    }
}

/** Run the skeleton extraction pass */
void CorridorPlanner::extractSkeleton()
{
    if (isProcessingRequired(SKELETON))
    {
        voronoi_points = skeleton->processDStar(
                dstar_to_start->graph(), dstar_to_goal->graph(),
                m_current.x(), m_current.y(), m_expand);
        processed();
    }
}

/** Run the plan computation pass */
void CorridorPlanner::computePlan()
{
    if (isProcessingRequired(PLAN))
    {
        plan = skeleton->buildPlan(PointID(m_current.x(), m_current.y()), PointID(m_goal.x(), m_goal.y()), dstar_to_goal->graph(), voronoi_points);
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
        plan.simplify(m_expand, lround(min_width / map->getScale()));
        exportPlan();
        processed();
    }
}

template<typename SrcContainer, typename DstContainer>
static void wrapContainer(DstContainer& dest, SrcContainer& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    dest.resize(src.size());

    size_t point_idx;
    typename SrcContainer::iterator src_it = src.begin();
    typename SrcContainer::iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src.end(); ++point_idx, ++src_it)
    {
        toWrapper(dest[point_idx], *src_it, scale, raster_to_world);
    }
}

template<int DIM, typename Transform>
static void toWrapper(base::geometry::Spline<DIM>& dest, base::geometry::Spline<DIM> const& src,
        Transform const& raster_to_world)
{
    dest = src;
    dest.transform(raster_to_world);
}

static void toWrapper(corridors::Corridor& dest, Corridor& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    src.updateCurves();
    toWrapper(dest.median_curve, src.median_curve, raster_to_world);
    toWrapper(dest.boundary_curves[0], src.boundary_curves[0], raster_to_world);
    toWrapper(dest.boundary_curves[1], src.boundary_curves[1], raster_to_world);

    dest.min_width = src.min_width;
    dest.max_width = src.max_width;
    toWrapper(dest.width_curve, src.width_curve, scale);
}

static void toWrapper(corridors::Plan& dest, Plan& src,
        double scale, Eigen::Transform3d const& raster_to_world)
{
    wrapContainer(dest.corridors, src.corridors, scale, raster_to_world);

    for (unsigned int corridor_idx = 0; corridor_idx < src.corridors.size(); ++corridor_idx)
    {
        Corridor const& corridor = src.corridors[corridor_idx];
        Corridor::const_connection_iterator
            conn_it = corridor.connections.begin(),
            conn_end = corridor.connections.end();

        for (; conn_it != conn_end; ++conn_it)
        {
            corridors::CorridorConnection conn = 
                { corridor_idx, conn_it->this_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE,
                  conn_it->target_idx, conn_it->target_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE };

            dest.connections.push_back(conn);
        }
    }

    dest.start_corridor = src.findStartCorridor();
    dest.end_corridor   = src.findEndCorridor();
}


void CorridorPlanner::exportPlan()
{
    Eigen::Transform3d raster_to_world(map->getLocalToWorld());
    toWrapper(final, plan, map->getScale(), raster_to_world);
}

/** Do all the passes in the right order */
corridors::Plan const& CorridorPlanner::result() const
{ return final; }

/** Do all the passes in the right order */
corridors::Plan const& CorridorPlanner::compute()
{
    computeDStar();
    extractSkeleton();
    computePlan();
    simplifyPlan();
    return final;
}

