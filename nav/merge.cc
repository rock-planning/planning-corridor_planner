#include "merge.hh"
#include <cmath>
#include <stdexcept>
#include <iostream>

using namespace Nav;
using namespace std;
using namespace boost;

PlanMerge::PlanMerge()
    : mode(NONE) {}

void PlanMerge::merge(Plan const& left, Plan const& right)
{
    // First, concatenate the two set of corridors
    corridors = left.corridors;
    concat(right);
}

bool PlanMerge::mergeCorridors(int left_idx, int right_idx,
        float coverage_threshold, float angular_threshold)
{
    if (!corridors[left_idx].bbox.intersects(corridors[right_idx].bbox))
        return false;

    mode = NONE;
    accumulator.clear();
    accumulator_point_mapping.clear();
    point_mapping.clear();
    merged_points.clear();
    endpoints.clear();

    bool did_merge = false;
    float cos_angular_threshold = cos(angular_threshold);

    Corridor left  = corridors[left_idx];
    Corridor right = corridors[right_idx];

    // First, traverse the left corridor and build left-only and merged
    // corridors
    for (MedianLine::const_iterator left_slice = left.median.begin(); left_slice != left.median.end(); ++left_slice)
    {
        PointID left_p = left_slice->first;
        // Find the median point in +right+ that is closest to left_p

        MedianLine::const_iterator min_slice = right.median.begin();
        MedianLine::const_iterator right_slice = min_slice;
        PointID right_p = right_slice->first;
        float distance = left_p.distance2(right_p);
        for (; min_slice != right.median.end(); ++min_slice)
        {
            PointID p   = min_slice->first;
            float d = left_p.distance2(p);
            if (d < distance)
            {
                right_p = p;
                right_slice = min_slice;
                distance = d;
            }
        }
        distance = sqrt(distance);

        // Now, check the principal direction of both the points
        Point<float> left_dir  = left_slice->second.direction();
        Point<float> right_dir = right_slice->second.direction();

        if (left_dir * right_dir < cos_angular_threshold)
        {
            pushSingle(left, left_slice);
            continue;
        }

        // OK, the two slices are more or less parallel. Check that they also
        // intersect
        float left_w = left_slice->second.width;
        float right_w = right_slice->second.width;

        if ((right_w - (left_w - distance)) / right_w > 1 - coverage_threshold)
            pushSingle(left, left_slice);
        else if ((left_w - (right_w - distance)) / left_w > 1 - coverage_threshold)
            pushSingle(left, left_slice);
        else
        {
            did_merge = true;
            pushMerged(left, left_slice, right, right_slice,
                    left_slice->first, left_slice->second);
        }
    }

    // No merge detected, just stop here since there is changes needed
    if (!did_merge)
        return false;

    // Push the leftovers from the first pass
    pushAccumulator();

    // Now that we did everything we could for the left corridor, let's traverse
    // the right corridor. Here, we just have to register all points that are
    // not already merged since the merging is symmetric
    mode = NONE;
    for (MedianLine::const_iterator right_slice = right.median.begin(); right_slice != right.median.end(); ++right_slice)
    {
        if (merged_points.count(right_slice->first) && mode == SINGLE)
        {
            if (accumulator.median.size() > 1) // to account for rasterization errors
                pushAccumulator();
            accumulator.clear();
            mode = NONE;
        }
        else
        {
            pushSingle(right, right_slice);
            mode = SINGLE;
        }
    }
    if (mode == SINGLE && accumulator.median.size() > 1)
        pushAccumulator();

    finalizeMerge();

    return true;
}

void PlanMerge::pushAccumulator()
{
    for (list<AccMappingTuple>::const_iterator it = accumulator_point_mapping.begin(); it != accumulator_point_mapping.end(); ++it)
    {
        PointID source_p, target_p;
        Corridor const* source_corridor;
        tie(source_p, source_corridor, target_p) = *it;

        if (mode == MERGING)
            merged_points.insert(source_p);

        if (source_p != target_p)
            point_mapping.push_back( make_tuple(source_p, corridors.size(), target_p) );

        copyConnections(corridors.size(), accumulator, target_p, *source_corridor, source_p);
    }

    // Register the endpoints to connect the corridors to each other afterwards
    endpoints.push_back( make_tuple(accumulator.median.begin()->first, corridors.size(), mode == MERGING) );
    endpoints.push_back( make_tuple(accumulator.median.rbegin()->first, corridors.size(), mode == MERGING) );
    corridors.push_back(accumulator);
    accumulator_point_mapping.clear();
    accumulator.clear();
}

void PlanMerge::pushSingle(Corridor const& corridor, MedianLine::const_iterator point)
{
    if (mode == MERGING)
        pushAccumulator();
    mode = SINGLE;

    // Add both the points and the connections that go from/to that point. If
    // there is connections, also update the other end of it
    accumulator_point_mapping.push_back(make_tuple(point->first, &corridor, point->first));
    accumulator.add(*point);
}

void PlanMerge::copyConnections(int target_idx, Corridor& target, PointID const& target_p,
        Corridor const& source, PointID const& source_p)
{
    for (Corridor::Connections::const_iterator it = source.connections.begin(); it != source.connections.end(); ++it)
    {
        if (it->get<0>() != source_p)
            continue;

        int       edge_idx = it->get<1>();
        PointID   edge_p   = it->get<2>();
        Corridor& edge_endpoint = corridors[edge_idx];

        target.connections.push_back( make_tuple(target_p, edge_idx, edge_p) );

        for (Corridor::Connections::iterator edge_it = edge_endpoint.connections.begin();
                edge_it != edge_endpoint.connections.end(); ++edge_it)
        {
            if (edge_it->get<0>() == edge_p && edge_it->get<2>() == source_p)
            {
                edge_it->get<1>() = target_idx;
                edge_it->get<2>() = target_p;
            }
        }
    }
}

void PlanMerge::pushMerged(
        Corridor const& left,  MedianLine::const_iterator left_p,
        Corridor const& right, MedianLine::const_iterator right_p,
        PointID const& p, MedianPoint const& median)
{
    if (mode == SINGLE)
        pushAccumulator();
    mode = MERGING;

    accumulator_point_mapping.push_back(make_tuple(left_p->first, &left, p));
    accumulator_point_mapping.push_back(make_tuple(right_p->first, &right, p));
    accumulator.add(p, median);
}

void PlanMerge::finalizeMerge()
{
    for (EndPoints::const_iterator src = endpoints.begin(); src != endpoints.end(); ++src)
    {
        PointID src_p   = src->get<0>();
        int     src_idx = src->get<1>();
        bool    src_is_merge = src->get<2>();

        // Check if there are other endpoints that are closeby this one
        EndPoints::const_iterator trg = src;
        for (++trg; trg != endpoints.end(); ++trg)
        {
            if (trg->get<2>() == src_is_merge)
                continue;

            PointID trg_p = trg->get<0>();
            int     trg_idx = trg->get<1>();
            if (trg_p.isNeighbour(src_p) && trg_idx != src_idx)
            {
                corridors[src_idx].connections.push_back( make_tuple(src_p, trg_idx, trg_p) );
                corridors[trg_idx].connections.push_back( make_tuple(trg_p, src_idx, src_p) );
            }
        }

        // Now, also check for merged points if src is a non-merge corridor
        if (src_is_merge)
            continue;

        for (list<PtMappingTuple>::const_iterator trg = point_mapping.begin(); trg != point_mapping.end(); ++trg)
        {
            int trg_idx = trg->get<1>();
            if (trg->get<0>().isNeighbour(src_p) && trg_idx != src_idx)
            {
                corridors[src_idx].connections.push_back( make_tuple(src_p, trg_idx, trg->get<2>()) );
                corridors[trg_idx].connections.push_back( make_tuple(trg->get<2>(), src_idx, src_p) );
            }
        }
    }
}

