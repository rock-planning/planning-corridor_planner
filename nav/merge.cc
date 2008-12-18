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
    bool did_merge = false;
    float cos_angular_threshold = cos(angular_threshold);

    Corridor left  = corridors[left_idx];
    Corridor right = corridors[right_idx];
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
            pushLeftRight(left, left_slice, right, right_slice);
            continue;
        }

        // OK, the two slices are more or less parallel. Check that they also
        // intersect
        float left_w = left_slice->second.width;
        float right_w = right_slice->second.width;

        if ((right_w - (left_w - distance)) / right_w > 1 - coverage_threshold)
            pushLeftRight(left, left_slice, right, right_slice);
        else if ((left_w - (right_w - distance)) / left_w > 1 - coverage_threshold)
            pushLeftRight(left, left_slice, right, right_slice);
        else
        {
            did_merge = true;
            pushMerged(left, left_slice, right, right_slice,
                    left_slice->first, left_slice->second);
        }
    }

    if (did_merge)
    {
        finalizeMerge();
        return true;
    }
    else return false;
}

void PlanMerge::pushLeftRight(Corridor const& left, MedianLine::const_iterator left_p,
        Corridor const& right, MedianLine::const_iterator right_p)
{
    if (mode == MERGING)
    {
        corridors.push_back(merged_corridor);
        left_corridor.clear();
        right_corridor.clear();
    }
    mode = LEFT_RIGHT;

    // Add both the points and the connections that go from/to that point. If
    // there is connections, also update the other end of it
    left_corridor.add(*left_p);
    copyConnections(corridors.size(), left_corridor, left_p->first, left, left_p->first);
    right_corridor.add(*right_p);
    copyConnections(corridors.size() + 1, right_corridor, right_p->first, right, right_p->first);
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
    if (mode == LEFT_RIGHT)
    {
        corridors.push_back(left_corridor);
        corridors.push_back(right_corridor);
        merged_corridor.clear();
    }
    mode = MERGING;

    merged_corridor.add(p, median);
    copyConnections(corridors.size(), merged_corridor, p, left, left_p->first);
    copyConnections(corridors.size(), merged_corridor, p, right, right_p->first);
}

void PlanMerge::finalizeMerge()
{
    if (mode == LEFT_RIGHT)
    {
        corridors.push_back(left_corridor);
        corridors.push_back(right_corridor);
    }
    else if (mode == MERGING)
        corridors.push_back(merged_corridor);

    left_corridor.clear();
    right_corridor.clear();
    merged_corridor.clear();
    
    mode = NONE;
}

