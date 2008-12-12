#include "merge.hh"
#include <cmath>
#include <stdexcept>

using namespace Nav;
using namespace std;

PlanMerge::PlanMerge()
    : mode(NONE) {}

void PlanMerge::merge(Plan const& left, Plan const& right)
{
    // First, concatenate the two set of corridors
    corridors = left.corridors;
    concat(right);
}

void PlanMerge::mergeCorridors(Corridor const& left, Corridor const& right, vector<Corridor>& result,
        float coverage_threshold, float angular_threshold)
{
    if (!left.bbox.intersects(right.bbox))
        return;

    float cos_angular_threshold = cos(angular_threshold);
    for (MedianLine::const_iterator left_slice = left.median.begin(); left_slice != left.median.end(); ++left_slice)
    {
        PointID left_p = left_slice->first;
        // Find the median point in +right+ that is closest to left_p

        MedianLine::const_iterator min_slice = left.median.begin();
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

        if (left_dir * right_dir > cos_angular_threshold)
        {
            pushLeftRight(left, left_slice, right, right_slice);
            continue;
        }

        // OK, the two slices are more or less parallel. Check that they also
        // intersect
        float left_w = left_slice->second.width;
        float right_w = min_slice->second.width;

        if ((right_w - (left_w - distance)) / right_w > 1 - coverage_threshold)
            pushLeftRight(left, left_slice, right, right_slice);
        else if ((left_w - (right_w - distance)) / left_w > 1 - coverage_threshold)
            pushLeftRight(left, left_slice, right, right_slice);
        else
            pushMerged(left, left_slice, right, right_slice,
                    left_slice->first, left_slice->second);
    }

    finalizeMerge();
}

void PlanMerge::pushLeftRight(Corridor const& left, MedianLine::const_iterator left_p,
        Corridor const& right, MedianLine::const_iterator right_p)
{
    if (mode == MERGING)
    {
        corridors.push_back(merged_corridor);
        merged_corridor.clear();
        mode = LEFT_RIGHT;
    }

    // Add both the points and the connections that go from/to that point. If
    // there is connections, also update the other end of it
    pushPoint(corridors.size(), left_corridor, left, left_p);
    pushPoint(corridors.size() + 1, right_corridor, right, right_p);
}

void PlanMerge::pushPoint(int target_idx, Corridor& target, Corridor const& source, MedianLine::const_iterator median)
{
    target.add(*median);

    PointID p = median->first;
    for (Corridor::Connections::const_iterator it = source.connections.begin(); it != source.connections.end(); ++it)
    {
        if (it->get<0>() != p)
            continue;
        PointID target_p = it->get<2>();

        target.connections.push_back(*it);
        int edge_idx = it->get<1>();

        Corridor& edge_endpoint = corridors[edge_idx];
        for (Corridor::Connections::iterator edge_it = edge_endpoint.connections.begin();
                edge_it != edge_endpoint.connections.end(); ++edge_it)
        {
            if (edge_it->get<0>() == target_p && edge_it->get<2>() == p)
                edge_it->get<1>() = target_idx;
        }
    }
}

void PlanMerge::pushMerged(Corridor const& left, MedianLine::const_iterator left_p,
        Corridor const& right, MedianLine::const_iterator right_p,
        PointID const& p, MedianPoint const& median)
{
    throw std::runtime_error("not implemented");
}

void PlanMerge::finalizeMerge()
{
    throw std::runtime_error("not implemented");
}

