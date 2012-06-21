#include "annotations.hh"
#include <envire/maps/MLSGrid.hpp>

using namespace corridor_planner;
using corridors::Corridor;
using envire::MLSGrid;

AnnotationFilter::AnnotationFilter(std::string const& symbol)
    : symbol(symbol) {}

void AnnotationFilter::apply(AnnotationFilter& filter, corridors::Plan& plan)
{
    int annotation_idx = plan.findAnnotationIndex(filter.symbol);
    if (annotation_idx == -1)
    {
        annotation_idx = plan.annotation_symbols.size();
        plan.annotation_symbols.push_back(filter.symbol);
    }

    for (unsigned int corridor_idx = 0; corridor_idx < plan.corridors.size(); ++corridor_idx)
    {
        corridors::Corridor& c = plan.corridors[corridor_idx];
        for (unsigned int curve_idx = 0; curve_idx < 3; ++curve_idx)
        {
            if (static_cast<int>(c.annotations[curve_idx].size()) <= annotation_idx)
                c.annotations[curve_idx].resize(annotation_idx + 1);
        }
        filter.annotate(annotation_idx, c);
    }
}

StrongEdgeAnnotation::StrongEdgeAnnotation(envire::Grid<float> const* step_map, std::string const& band_name, double threshold)
    : AnnotationFilter("STRONG_EDGE"), map(step_map), band(band_name), threshold(threshold) {}


void StrongEdgeAnnotation::annotateCurve(corridors::Corridor::Annotations& result, base::geometry::Spline<3> const& curve)
{
    typedef corridors::Corridor::AnnotatedSegment AnnotatedSegment;
    double const scaleX = map->getScaleX(), scaleY = map->getScaleY();
    size_t const map_width = map->getWidth(), map_height = map->getHeight();

    std::vector<double> parameters;
    std::vector<base::Vector3d> points = curve.sample(std::min(scaleX, scaleY) / 2, &parameters);

    // The current state
    double state_start = curve.getStartParam();
    int current_state = -1;

    boost::multi_array<float, 2> const& data = map->getGridData(band);
    std::pair<float, bool> const no_data = map->getNoData(band);

    static const int RADIUS = ceil(0.8 / scaleX);

    envire::FrameNode const* world_node = map->getFrameNode()->getRoot();
    for (unsigned int point_idx = 0; point_idx < points.size(); ++point_idx)
    {
        size_t x, y;
        map->toGrid(points[point_idx], x, y, world_node);
        if (x <= (size_t)RADIUS || x >= map_width - RADIUS || y <= (size_t)RADIUS || y >= map_height - RADIUS)
            continue;

        int new_state = -1;
        for (int dy = -RADIUS; new_state != STRONG && dy < RADIUS + 1; ++dy)
        {
            for (int dx = -RADIUS; dx < RADIUS + 1; ++dx)
            {
                double cell_data = data[y + dy][x + dx];
                if (no_data.second && cell_data == no_data.first && new_state == -1)
                {
                    new_state = UNKNOWN;
                }
                else if (cell_data > threshold)
                {
                    new_state = STRONG;
                    break;
                }
            }
        }

        if (new_state != current_state)
        {
            if (current_state != -1)
                result.push_back( AnnotatedSegment(state_start, parameters[point_idx], current_state) );

            state_start = parameters[point_idx];
        }
        current_state = new_state;
    }
    if (current_state != -1)
    {
        result.push_back( AnnotatedSegment(state_start, parameters.back(), current_state) );
    }
}

void StrongEdgeAnnotation::annotate(int index, corridors::Corridor& corridor)
{
    annotateCurve(corridor.annotations[1][index], corridor.boundary_curves[0]);
    annotateCurve(corridor.annotations[2][index], corridor.boundary_curves[1]);
}


NarrowWideAnnotation::NarrowWideAnnotation(double narrow_threshold, double wide_threshold)
    : AnnotationFilter("NARROW_WIDE"), narrow_threshold(narrow_threshold), wide_threshold(wide_threshold)
{
}
void NarrowWideAnnotation::annotate(int index, corridors::Corridor& corridor)
{
    typedef base::geometry::Spline<1> Spline;
    typedef corridors::Corridor::AnnotatedSegment AnnotatedSegment;

    Spline& width_curve = corridor.width_curve;
    std::vector<double> parameters;

    double sample_step;
    if (narrow_threshold > 0)
        sample_step = narrow_threshold / 10;
    else if (wide_threshold > 0)
        sample_step = wide_threshold / 10;
    else
        throw std::runtime_error("both the narrow_threshold and wide_threshold parameters are zero !");

    std::vector< Spline::vector_t > points = width_curve.sample(sample_step, &parameters);

    corridors::Corridor::Annotations& result =
        corridor.annotations[0][index];

    // The current state
    double state_start = corridor.width_curve.getStartParam();
    int current_state = -1;
    for (size_t point_idx = 0; point_idx < points.size(); ++point_idx)
    {
        int new_state = -1;
        if (points[point_idx][0] < narrow_threshold)
            new_state = NARROW;
        else if (points[point_idx][0] > wide_threshold)
            new_state = WIDE;

        if (new_state != current_state)
        {
            if (current_state != -1)
                result.push_back( AnnotatedSegment(state_start, parameters[point_idx], current_state) );

            state_start = parameters[point_idx];
        }
        current_state = new_state;
    }

    if (current_state != -1)
    {
        result.push_back( AnnotatedSegment(state_start, parameters.back(), current_state) );
    }
}

KnownUnknownAnnotation::KnownUnknownAnnotation(envire::Grid<uint8_t> const* map,
        std::string const& band_name, uint8_t unknown_class)
    : AnnotationFilter("KNOWN_UNKNOWN")
    , map(map), band_name(band_name), unknown_class(unknown_class)
{
}

void KnownUnknownAnnotation::annotate(int index, corridors::Corridor& corridor)
{
    typedef base::geometry::Spline<3> Spline;
    typedef corridors::Corridor::AnnotatedSegment AnnotatedSegment;

    std::vector<double> parameters;

    envire::Grid<uint8_t>::ArrayType const& data =
        map->getGridData(band_name);

    double sample_step = std::min(map->getScaleX(), map->getScaleY()) / 2;
    std::vector<base::Vector3d> points =
        corridor.median_curve.sample(sample_step, &parameters);

    corridors::Corridor::Annotations& result =
        corridor.annotations[0][index];
    envire::FrameNode const* world_node = map->getFrameNode()->getRoot();

    // The current state
    double state_start = corridor.median_curve.getStartParam();
    int current_state = -1;
    for (size_t point_idx = 0; point_idx < points.size(); ++point_idx)
    {
        size_t x, y;
        map->toGrid(points[point_idx], x, y, world_node);

        int new_state = KNOWN;
        if (data[y][x] == unknown_class)
            new_state = UNKNOWN;

        if (new_state != current_state)
        {
            if (current_state != -1)
                result.push_back( AnnotatedSegment(state_start, parameters[point_idx], current_state) );

            state_start = parameters[point_idx];
        }
        current_state = new_state;
    }

    if (current_state != -1)
    {
        result.push_back( AnnotatedSegment(state_start, parameters.back(), current_state) );
    }
}

