#include "corridor_planner.hh"

#include "skeleton.hh"
#include "voronoi.hh"
#include "annotations.hh"
#include "plan.hh"

#include <stdexcept>


using namespace corridor_planner;

CorridorPlanner::CorridorPlanner()
    : map(0), skeleton(0), dstar_to_start(0), dstar_to_goal(0)
    , min_width(0)
    , env(0)
    , m_current(0, 0)
    , m_goal(0, 0)
    , m_state(DSTAR), m_expand(1.1)
    , strong_edge_enable(false), narrow_wide_enable(false), known_unknown_enable(false)
{
    requireProcessing(DSTAR);
}

CorridorPlanner::~CorridorPlanner()
{
    delete env;
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
void CorridorPlanner::init(std::string const& terrain_classes, std::string const& map_file, float min_width, float cost_cutoff)
{
    nav_graph_search::TerrainClasses classes = nav_graph_search::TerrainClass::load(terrain_classes);
    TraversabilityMap* map     = TraversabilityMap::load(map_file, classes);
    if (!map)
        throw std::runtime_error("cannot load the specified map");

    init(classes, map, min_width, cost_cutoff);
}

/** Load the terrain classes and traversability map */
void CorridorPlanner::init(std::string const& terrain_classes,
        envire::Grid<uint8_t> const& input_map, std::string const& band_name, float min_width, float cost_cutoff)
{
    nav_graph_search::TerrainClasses classes = nav_graph_search::TerrainClass::load(terrain_classes);
    TraversabilityMap* map     = TraversabilityMap::load(input_map, band_name, classes);
    if (!map)
        throw std::runtime_error("cannot load the specified map");

    init(classes, map, min_width, cost_cutoff);
}

void CorridorPlanner::init(nav_graph_search::TerrainClasses const& terrain_classes, TraversabilityMap* map, float min_width, float cost_cutoff)
{
    this->classes = terrain_classes;
    this->map = map;
    delete dstar_to_start;
    delete dstar_to_goal;
    
    map->writeToPPM("terrain_map");
    
    dstar_to_start = new DStar(*map, classes);
    dstar_to_start->setCostCutoff(cost_cutoff);
    dstar_to_goal  = new DStar(*map, classes);
    dstar_to_goal->setCostCutoff(cost_cutoff);
    delete skeleton;
    skeleton = new SkeletonExtraction(map->xSize(), map->ySize());

    this->min_width = min_width;
}

void CorridorPlanner::setMarginFactor(double factor)
{
    m_expand = factor;
    requireProcessing(DSTAR);
}

void CorridorPlanner::enableStrongEdgeFilter(std::string const& env_path, std::string map_id, std::string const& band_name, double threshold)
{
    delete env;
    env = envire::Environment::unserialize(env_path);
    strong_edge_map       = env->getItem< envire::Grid<float> >(map_id);
    if (!strong_edge_map)
        throw std::runtime_error("wrong map ID given for strong edge map");

    enableStrongEdgeFilter(strong_edge_map, band_name, threshold);
}

void CorridorPlanner::enableStrongEdgeFilter(envire::Grid<float>::Ptr step_size, std::string const& band_name, double threshold)
{
    strong_edge_enable    = true;
    strong_edge_map       = step_size;
    strong_edge_band      = band_name;
    strong_edge_threshold = threshold;
    requireProcessing(ANNOTATIONS);
}

void CorridorPlanner::disableStrongEdgeFilter()
{
    strong_edge_enable    = false;
    requireProcessing(ANNOTATIONS);
}

void CorridorPlanner::enableNarrowWideFilter(double narrow_threshold, double wide_threshold)
{
    narrow_wide_enable = true;
    narrow_wide_narrow_threshold = narrow_threshold;
    narrow_wide_wide_threshold   = wide_threshold;
    requireProcessing(ANNOTATIONS);
}

void CorridorPlanner::disableNarrowWideFilter()
{
    narrow_wide_enable = false;
    requireProcessing(ANNOTATIONS);
}

void CorridorPlanner::enableKnownUnknownFilter(envire::Grid<uint8_t>::Ptr map, std::string const& band_name, uint8_t unknown_class)
{
    known_unknown_enable    = true;
    known_unknown_map       = map;
    known_unknown_band      = band_name;
    known_unknown_class = unknown_class;
    requireProcessing(ANNOTATIONS);
}

void CorridorPlanner::disableKnownUnknownFilter()
{
    known_unknown_enable    = false;
    requireProcessing(ANNOTATIONS);
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
        requireProcessing(DSTAR);
    }
    if (goal != m_goal)
    {
        m_goal = goal;
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
        double optimal = dstar_to_start->run(m_current.x(), m_current.y(), m_goal.x(), m_goal.y());
        if (base::isUnknown<double>(optimal))
            throw CostCutoffReached("reached cost cutoff (" + boost::lexical_cast<std::string>(dstar_to_start->getCostCutoff()) + ") during planning from goal to start");
	dstar_to_start->writeCostMap("costMap_toStart_optimal");

        std::cout << "to start: optimal before expansion is " << optimal << std::endl;
        dstar_to_start->expandUntil(optimal * m_expand);
	dstar_to_start->writeCostMap("costMap_toStart_expanded");

        optimal = dstar_to_goal->run(m_goal.x(), m_goal.y(), m_current.x(), m_current.y());
        if (base::isUnknown<double>(optimal))
            throw CostCutoffReached("reached cost cutoff (" + boost::lexical_cast<std::string>(dstar_to_start->getCostCutoff()) + ") during planning from start to goal");
	dstar_to_goal->writeCostMap("costMap_toGoal_optimal");

        std::cout << "to goal:  optimal before expansion is " << optimal << std::endl;
        dstar_to_goal->expandUntil(optimal * m_expand);
        std::cout << "to start: optimal after expansion is " << dstar_to_start->graph().getValue(m_goal.x(), m_goal.y()) << std::endl;
        std::cout << "to goal:  optimal after expansion is " << dstar_to_goal->graph().getValue(m_current.x(), m_current.y()) << std::endl;
	dstar_to_goal->writeCostMap("costMap_toGoal_expanded");
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
        processed();
    }
}

/** Run the annotation pass on the generated corridors
 */
void CorridorPlanner::annotateCorridors()
{
    if (isProcessingRequired(ANNOTATIONS))
    {
        exportPlan();

        if (strong_edge_enable)
        {
            StrongEdgeAnnotation filter(strong_edge_map.get(), strong_edge_band, strong_edge_threshold);
            AnnotationFilter::apply(filter, final);
        }
        if (narrow_wide_enable)
        {
            NarrowWideAnnotation filter(narrow_wide_narrow_threshold, narrow_wide_wide_threshold);
            AnnotationFilter::apply(filter, final);
        }
        if (known_unknown_enable)
        {
            KnownUnknownAnnotation filter(known_unknown_map.get(), known_unknown_band, known_unknown_class);
            AnnotationFilter::apply(filter, final);
        }
        processed();
    }
}

/** Called when the planning is finished
 */
void CorridorPlanner::done()
{
}

template<typename SrcContainer, typename DstContainer>
static void wrapContainer(DstContainer& dest, SrcContainer& src,
        double scale, Eigen::Affine3d const& raster_to_world);

template<int DIM, typename Transform>
static void toWrapper(base::geometry::Spline<DIM>& dest, base::geometry::Spline<DIM> const& src,
        Transform const& raster_to_world)
{
    dest = src;
    dest.transform(raster_to_world);
}

static void toWrapper(corridors::Corridor& dest, Corridor& src,
        double scale, Eigen::Affine3d const& raster_to_world)
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
        double scale, Eigen::Affine3d const& raster_to_world)
{
    wrapContainer(dest.corridors, src.corridors, scale, raster_to_world);

    dest.connections.clear();

    for (unsigned int corridor_idx = 0; corridor_idx < src.corridors.size(); ++corridor_idx)
    {
        Corridor const& corridor = src.corridors[corridor_idx];
        Corridor::const_connection_iterator
            conn_it = corridor.connections.begin(),
            conn_end = corridor.connections.end();

        dest.connections.reserve(dest.connections.size() + corridor.connections.size());
        for (; conn_it != conn_end; ++conn_it)
        {
            corridors::CorridorConnection conn = 
                { static_cast<int>(corridor_idx), conn_it->this_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE,
                  conn_it->target_idx, conn_it->target_side ? corridors::BACK_SIDE : corridors::FRONT_SIDE };

            dest.connections.push_back(conn);
        }
    }

    dest.start_corridor = src.findStartCorridor();
    dest.end_corridor   = src.findEndCorridor();
}

template<typename SrcContainer, typename DstContainer>
static void wrapContainer(DstContainer& dest, SrcContainer& src,
        double scale, Eigen::Affine3d const& raster_to_world)
{
    dest.resize(src.size());

    size_t point_idx;
    typename SrcContainer::iterator src_it = src.begin();
    typename SrcContainer::iterator const src_end = src.end();
    for (point_idx = 0, src_it = src.begin(); src_it != src_end; ++point_idx, ++src_it)
    {
        toWrapper(dest[point_idx], *src_it, scale, raster_to_world);
    }
}

void CorridorPlanner::exportPlan()
{
    Eigen::Affine3d raster_to_world(map->getLocalToWorld());
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
    annotateCorridors();
    done();
    return final;
}

