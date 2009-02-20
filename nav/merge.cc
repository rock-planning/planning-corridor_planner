#include "merge.hh"
#include <cmath>
#include <stdexcept>
#include <iostream>

using namespace Nav;
using namespace std;
using namespace boost;

PlanMerge::PlanMerge()
    : mode(NONE) {}

void PlanMerge::merge(Plan const& left, Plan const& right, float coverage_threshold, float angular_threshold)
{
    // First, concatenate the two set of corridors
    corridors = left.corridors;
    concat(right);

    m_start = left.getStartPoint();
    m_end   = left.getEndPoint();
    m_nav_function = left.getNavigationFunction();

    ownership.resize(corridors.size());
    fill(ownership.begin(), ownership.begin() + left.corridors.size(), LEFT_SIDE);
    fill(ownership.begin() + left.corridors.size(), ownership.end(), RIGHT_SIDE);
    process(coverage_threshold, angular_threshold);
}

void PlanMerge::process(float coverage_threshold, float angular_threshold)
{
    point_mapping.clear();
    int original_corridor_size = corridors.size();
    for (int i = 0; i < original_corridor_size; ++i)
    {
        if (ownership[i] == TO_DELETE)
            continue;

        cerr << "looking at corridor " << i << endl;

        int original_corridors_size = corridors.size();
        bool did_merge = false;
        size_t j = i + 1;
        for (; j < corridors.size(); ++j)
        {
            //cerr << "merging " << i << " " << j << endl;
            if (ownership[j] == ownership[i] || ownership[j] == TO_DELETE) continue;
            int left_idx = i, right_idx = j;
            if (ownership[i] == RIGHT_SIDE)
                swap(left_idx, right_idx);

            if ((did_merge = mergeCorridors(left_idx, right_idx, coverage_threshold, angular_threshold)))
                break;
        }

        if (did_merge)
        {
            cerr << "merged " << i << " with " << j << " into [" << original_corridors_size << ", " << corridors.size() << "]" << endl;
            // A merge happened, remove the original corridors and adjust the
            // indexes
            ownership[i] = ownership[j] = TO_DELETE;
        }
    }

    copyConnections();
    for (size_t i = 1, owner_i = 1; i < corridors.size(); ++owner_i)
    {
        if (ownership[owner_i] == TO_DELETE)
            removeCorridor(i);
        else
            ++i;
    }

    mergeSimpleCrossroads();
}

bool PlanMerge::mergeCorridors(int left_idx, int right_idx,
        float coverage_threshold, float angular_threshold)
{
    if (!corridors[left_idx].bbox.intersects(corridors[right_idx].bbox))
        return false;

    mode = NONE;
    accumulator.clear();
    accumulated_point_mappings.clear();
    merging_endpoints.clear();
    merged_points.clear();

    bool did_merge = false;
    float cos_angular_threshold = cos(angular_threshold);
    cerr << " angular limit: " << angular_threshold << " (cos=" << cos_angular_threshold << endl;

    // There is a copy here because of an initial (bad) decision to refer to
    // other corridors using indexes. Therefore, when we add new corridors to
    // vector<>, left and right may get invalidated ...
    Corridor left  = corridors[left_idx];
    Corridor right = corridors[right_idx];

    // First, traverse the left corridor and build left-only and merged
    // corridors
    current_owner = LEFT_SIDE; // used by pushAccumulator to mark the ownership of new corridors
    for (MedianLine::const_iterator left_slice = left.median.begin(); left_slice != left.median.end(); ++left_slice)
    {
        PointID left_p = left_slice->center;
        // Find the median point in +right+ that is closest to left_p

        MedianLine::const_iterator min_slice = right.median.begin();
        MedianLine::const_iterator right_slice = min_slice;
        PointID right_p = right_slice->center;
        float distance = left_p.distance2(right_p);
        for (; min_slice != right.median.end(); ++min_slice)
        {
            PointID p   = min_slice->center;
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
        Point<float> left_dir  = left_slice->direction();
        Point<float> right_dir = right_slice->direction();

        float cos_angle = fabs(left_dir * right_dir);
        cerr << left_p << " " << right_p << " left_dir=" << left_dir << " right_dir=" << right_dir << "\n"
            << " angle=" << cos_angle << "\n";

        if (cos_angle < cos_angular_threshold)
        {
            pushSingle(left_idx, left_slice);
            continue;
        }

        // Now, check two things
        //  - that the two corridors are near to each other (perpendicularly to
        //    the corridor main direction)
        //  - that the two corridors overlap (along the corridor main direction)
        Point<float> dist_dir = left_p - right_p;
        float para_distance  = fabs(dist_dir * left_dir);
        float ortho_distance = fabs(sqrt(distance * distance - para_distance * para_distance));

        cerr << " d=" << distance << " para_distance=" << para_distance << " ortho_distance=" << ortho_distance << endl;

        if (ortho_distance > 3)
        {
            pushSingle(left_idx, left_slice);
            continue;
        }

        // OK, the two slices are more or less parallel. Check that they also
        // intersect
        float left_w = left_slice->width;
        float right_w = right_slice->width;

        if ((right_w - (left_w - para_distance)) / right_w > 1 - coverage_threshold)
            pushSingle(left_idx, left_slice);
        else if ((left_w - (right_w - para_distance)) / left_w > 1 - coverage_threshold)
            pushSingle(left_idx, left_slice);
        else
        {
            did_merge = true;
            pushMerged(left_idx, left_slice, right_idx, right_slice,
                    left_slice->center, *left_slice);
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
    current_owner = RIGHT_SIDE; // used by pushAccumulator to mark the ownership of new corridors
    for (MedianLine::const_iterator right_slice = right.median.begin(); right_slice != right.median.end(); ++right_slice)
    {
        if (merged_points.count(right_slice->center))
        {
            if (mode == SINGLE)
            {
                if (accumulator.median.size() > 1) // to account for rasterization errors
                    pushAccumulator();
                accumulated_point_mappings.clear();
                accumulator.clear();
                mode = NONE;
            }
        }
        else
        {
            pushSingle(right_idx, right_slice);
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
    merging_endpoints.push_back(last_point[0]);
    if (mode == MERGING)
    {
        merging_endpoints.push_back(last_point[1]);
        ownership.push_back(MERGED);
    }
    else
        ownership.push_back(current_owner);

    point_mapping.insert( accumulated_point_mappings.begin(), accumulated_point_mappings.end() );
    corridors.push_back(accumulator);
    accumulated_point_mappings.clear();
    accumulator.clear();
}

void PlanMerge::pushSingle(int source_idx, MedianLine::const_iterator point)
{
    PointID median_point = point->center;
    PtMappingTuple new_mapping = make_tuple(source_idx, median_point, corridors.size(), median_point);

    cerr << "NOT MERGING" << endl << endl;
    if (mode == MERGING)
    {
        pushAccumulator();
        merging_endpoints.push_back(new_mapping);
    }

    mode = SINGLE;

    // In point_mapping, we gather the mapping before the original point (in the
    // original corridor) and the point added to the accumulator. Both are, of
    // course, the same here but both points are different in case of a merge.
    accumulated_point_mappings[ make_pair(source_idx, point->center) ] = make_pair(corridors.size(), point->center);
    accumulator.add(*point);
    last_point[0] = new_mapping;
}

void PlanMerge::pushMerged(
        int left_idx,  MedianLine::const_iterator left_p,
        int right_idx, MedianLine::const_iterator right_p,
        PointID const& p, MedianPoint const& median)
{
    int new_idx = corridors.size();
    PointID left_point  = left_p->center;
    PointID right_point = right_p->center;
    PtMappingTuple from_left  = make_tuple(left_idx,  left_point,  corridors.size(), p);
    PtMappingTuple from_right = make_tuple(right_idx, right_point, corridors.size(), p);

    if (mode == SINGLE)
    {
        pushAccumulator();
        merging_endpoints.push_back(from_left);
        merging_endpoints.push_back(from_right);
    }

    mode = MERGING;
    accumulator.add(p, median);

    last_point[0] = from_left;
    merged_points.insert(left_point);
    accumulated_point_mappings[ make_pair(left_idx, left_point) ]   = make_pair(new_idx, p);
    last_point[1] = from_right;
    merged_points.insert(right_point);
    accumulated_point_mappings[ make_pair(right_idx, right_point) ] = make_pair(new_idx, p);
}

void PlanMerge::copyConnections()
{
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor::Connections& conn = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = conn.begin(); conn_it != conn.end(); ++conn_it)
        {
            PtMapping::const_iterator src_map = point_mapping.find( make_pair(i, conn_it->get<0>()) );
            if (src_map == point_mapping.end())
            {
                //if (ownership[i] == TO_DELETE)
                //    throw std::runtime_error("a connection cannot be copied");

                PtMapping::const_iterator dst_map = point_mapping.find( make_pair(conn_it->get<1>(), conn_it->get<2>()) );
                if (dst_map != point_mapping.end())
                {
                    conn_it->get<1>() = dst_map->second.first;
                    conn_it->get<2>() = dst_map->second.second;
                }
            }
            else
            {
                int new_idx = src_map->second.first;
                Corridor::Connections& new_conn = corridors[new_idx].connections;
                PointID   new_point = src_map->second.second;
                
                PtMapping::const_iterator dst_map = point_mapping.find( make_pair(conn_it->get<1>(), conn_it->get<2>()) );
                if (dst_map == point_mapping.end())
                    new_conn.push_back( make_tuple(new_point, conn_it->get<1>(), conn_it->get<2>()) );
                else
                    new_conn.push_back( make_tuple(new_point, dst_map->second.first, dst_map->second.second) );
            }
        }
    }
}

void PlanMerge::finalizeMerge()
{
    for (vector<PtMappingTuple>::const_iterator src = merging_endpoints.begin(); src != merging_endpoints.end(); ++src)
    {
        int     src_idx = src->get<2>();
        PointID src_p   = src->get<1>();

        // Check if there are other endpoints that are closeby this one
        vector<PtMappingTuple>::const_iterator trg = src;
        for (++trg; trg != merging_endpoints.end(); ++trg)
        {
            int     trg_idx = trg->get<2>();
            PointID trg_p   = trg->get<1>();
            if (trg_p.isNeighbour(src_p) && trg_idx != src_idx)
            {
                corridors[src_idx].connections.push_back( make_tuple(src->get<3>(), trg_idx, trg->get<3>()) );
                corridors[trg_idx].connections.push_back( make_tuple(trg->get<3>(), src_idx, src->get<3>()) );
            }
        }
    }
    merging_endpoints.clear();
}

