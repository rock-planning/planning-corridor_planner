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

StrongEdgeAnnotation::StrongEdgeAnnotation(envire::Grid<double> const* step_map, std::string const& band_name, double threshold)
    : AnnotationFilter("STRONG_EDGE"), map(step_map), band(band_name), threshold(threshold) {}


void StrongEdgeAnnotation::annotateCurve(corridors::Corridor::Annotations& result, base::geometry::Spline<3> const& curve)
{
    typedef corridors::Corridor::AnnotatedSegment AnnotatedSegment;
    double const scaleX = map->getScaleX(), scaleY = map->getScaleY();
    size_t const map_width = map->getWidth(), map_height = map->getHeight();

    std::vector<double> parameters;
    std::vector<base::Vector3d> points = curve.sample(std::min(scaleX, scaleY) / 2, &parameters);
    std::cout << "StrongEdgeAnnotation: marking " << points.size() << " points on curve fron band " << band << std::endl;

    // The current state
    double state_start = curve.getStartParam();
    int current_state = -1;

    boost::multi_array<double, 2> const& data = map->getGridData(band);
    std::pair<double, bool> const no_data = map->getNoData(band);

    envire::FrameNode const* world_node = map->getFrameNode()->getRoot();
    for (unsigned int point_idx = 0; point_idx < points.size(); ++point_idx)
    {
        size_t x, y;
        map->toGrid(points[point_idx], x, y, world_node);
        if (x == 0 || x == map_width - 1 || y == 0 || y == map_height - 1)
            continue;

        int new_state = -1;
        for (int dy = -1; dy < 2; ++dy)
            for (int dx = -1; dx < 2; ++dx)
            {
                if (no_data.second && data[y][x] == no_data.first && new_state == -1)
                    new_state = UNKNOWN;
                else if (data[y][x] > threshold)
                    new_state = STRONG;
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
        std::cout << "marking " << state_start << "->" << parameters.back() << " as " << current_state << std::endl;
        result.push_back( AnnotatedSegment(state_start, parameters.back(), current_state) );
    }
}

void StrongEdgeAnnotation::annotate(int index, corridors::Corridor& corridor)
{
    annotateCurve(corridor.annotations[1][index], corridor.boundary_curves[0]);
    annotateCurve(corridor.annotations[2][index], corridor.boundary_curves[1]);
}

