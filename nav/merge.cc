#include "merge.hh"
#include <cmath>
#include <stdexcept>
#include <iostream>

#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

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

    //for (size_t corr_idx = 0; corr_idx < corridors.size(); ++corr_idx)
    //{
    //    Corridor const& corridor = corridors[corr_idx];
    //    Corridor::Connections::const_iterator conn_it;
    //    for (conn_it = corridor.connections.begin(); conn_it != corridor.connections.end(); ++conn_it)
    //    {
    //        PointID p = conn_it->get<0>();
    //        cerr << corr_idx << " " << p << " " << conn_it->get<1>() << " " << conn_it->get<2>() << endl;
    //        if (find_if(corridor.median.begin(), corridor.median.end(),
    //                    bind(&MedianPoint::center, _1) == p) == corridor.median.end())
    //        {
    //            cerr << "connection point " << p << " is not in the median of " << corr_idx << endl;
    //        }
    //    }
    //}
    ownership.resize(corridors.size());
    fill(ownership.begin(), ownership.begin() + left.corridors.size(), LEFT_SIDE);
    fill(ownership.begin() + left.corridors.size(), ownership.end(), RIGHT_SIDE);
    process(coverage_threshold, angular_threshold);

    vector<int> useful_corridors;
    useful_corridors.resize(corridors.size(), 0);
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor const& corridor = corridors[i];
        if (corridor.isSingleton() &&
                (corridor.center == left.getStartPoint() || corridor.center == right.getStartPoint()
                 || corridor.center == left.getEndPoint() || corridor.center == right.getEndPoint()))
        {
            cerr << "corridor " << i << " is an endpoint corridor" << endl;
            useful_corridors[i] = USEFUL;
        }
    }
    //markUselessCorridors(useful_corridors);
    //removeUselessCorridors(useful_corridors);
    //mergeSimpleCrossroads_directed();
}

void PlanMerge::process(float coverage_threshold, float angular_threshold)
{
    size_t original_corridor_size = corridors.size();
    for (size_t i = 0; i < original_corridor_size; ++i)
    {
        if (ownership[i] == TO_DELETE)
            continue;

        cerr << "looking at corridor " << corridors[i].name << endl;

        int original_corridors_size = corridors.size();
        bool did_merge = false;
        size_t j;
        for (j = 0; j < corridors.size(); ++j)
        {
            //cerr << "merging " << i << " " << j << endl;
            if (ownership[j] == ownership[i] || ownership[j] == MERGED || ownership[j] == TO_DELETE)
                continue;
            int left_idx = i, right_idx = j;
            if (ownership[i] == RIGHT_SIDE)
                swap(left_idx, right_idx);

            if ((did_merge = mergeCorridors(left_idx, right_idx, coverage_threshold, angular_threshold)))
                break;
        }

        if (did_merge)
        {
            cerr << "merged " << corridors[i].name << " with " << corridors[j].name << " into [" << flush;
            for (size_t new_idx = original_corridors_size; new_idx < corridors.size(); ++new_idx)
                cerr << " " << corridors[new_idx].name;
            cerr << " ]" << endl;

            // A merge happened, remove the original corridors and adjust the
            // indexes
            ownership[i] = ownership[j] = TO_DELETE;
        }
    }

    for (size_t i = 0, owner_i = 0; i < corridors.size(); ++owner_i)
    {
        if (ownership[owner_i] == TO_DELETE)
            removeCorridor(i);
        else
            ++i;
    }
}

//pair<false,  PlanMerge::findMergeCandidate(MedianPoint const& left_slice, Corridor const& right,
//        float coverage_threshold, float cos_angular_threshold)
//{
//    PointID left_p = left_slice.center;
//
//    MedianLine::const_iterator min_slice   = corridor.median.begin();
//    MedianLine::const_iterator right_slice = min_slice;
//    PointID right_p = right_slice->center;
//    float distance  = left_p.distance2(right_p);
//    for (; min_slice != right.median.end(); ++min_slice)
//    {
//        PointID p = min_slice->center;
//        float   d = point.distance2(p);
//        if (d < distance)
//        {
//            right_p = p;
//            right_slice = min_slice;
//            distance = d;
//        }
//    }
//    distance = sqrt(distance);
//
//    // Now, check the principal direction of both the points
//    Point<float> left_dir  = left_slice.direction();
//    Point<float> right_dir = right_slice->direction();
//
//    float cos_angle = fabs(left_dir * right_dir);
//    //cerr << left_p << " " << right_p << " left_dir=" << left_dir << " right_dir=" << right_dir << "\n"
//    //    << " angle=" << cos_angle << "\n";
//
//    if (cos_angle < cos_angular_threshold)
//        return make_pair(false, MedianLine::const_iterator());
//
//    // Now, check two things
//    //  - that the two corridors are near to each other (perpendicularly to
//    //    the corridor main direction)
//    //  - that the two corridors overlap (along the corridor main direction)
//    Point<float> dist_dir = left_p - right_p;
//    float para_distance  = fabs(dist_dir * left_dir);
//    float ortho_distance = fabs(sqrt(distance * distance - para_distance * para_distance));
//
//    //cerr << " d=" << distance << " para_distance=" << para_distance << " ortho_distance=" << ortho_distance << endl;
//
//    if (ortho_distance > 3)
//        return make_pair(false, MedianLine::const_iterator());
//
//    // OK, the two slices are more or less parallel. Check that they also
//    // intersect
//    float left_w = left_slice->width;
//    float right_w = right_slice->width;
//
//    if ((right_w - (left_w - para_distance)) / right_w > 1 - coverage_threshold)
//        return make_pair(false, MedianLine::const_iterator());
//    else if ((left_w - (right_w - para_distance)) / left_w > 1 - coverage_threshold)
//        return make_pair(false, MedianLine::const_iterator());
//    else
//        return make_pair(true, right_slice);
//}

bool PlanMerge::canMerge(MedianPoint const& left_p, MedianPoint const& right_p,
        float coverage_threshold, float cos_angular_threshold) const
{
    return canMerge(left_p, right_p, sqrt(left_p.center.distance(right_p.center)), coverage_threshold, cos_angular_threshold);
}

bool PlanMerge::canMerge(MedianPoint const& left_slice, MedianPoint const& right_slice,
        float distance, float coverage_threshold, float cos_angular_threshold) const
{
    // Now, check the principal direction of both the points
    Point<float> left_dir  = left_slice.direction();
    Point<float> right_dir = right_slice.direction();

    PointID left_p  = left_slice.center;
    PointID right_p = right_slice.center;

    float cos_angle = fabs(left_dir * right_dir);
    //cerr << left_p << " " << right_p << " left_dir=" << left_dir << " right_dir=" << right_dir << "\n"
    //    << " angle=" << cos_angle << "\n";

    if (cos_angle < cos_angular_threshold)
        return false;

    // Now, check two things
    //  - that the two corridors are near to each other (perpendicularly to
    //    the corridor main direction)
    //  - that the two corridors overlap (along the corridor main direction)
    Point<float> dist_dir = left_p - right_p;
    float para_distance  = fabs(dist_dir * left_dir);
    float ortho_distance = fabs(sqrt(distance * distance - para_distance * para_distance));

    //cerr << " d=" << distance << " para_distance=" << para_distance << " ortho_distance=" << ortho_distance << endl;

    if (ortho_distance > 3)
        return false;

    // OK, the two slices are more or less parallel. Check that they also
    // intersect
    float w0 = left_slice.width;
    float w1 = right_slice.width;

    if (w1 > w0)
        swap(w0, w1);

    float diff = min<float>(para_distance + w1 - w0, 0);
    float coverage = (2 * w1 - diff) / (2 * w0);
    //cerr << "left_p=" << left_p << " right_p=" << right_p << " left_w=" << left_slice.width << " right_w=" << right_slice.width << " coverage=" << coverage << endl;
    if (coverage < coverage_threshold)
        return false;
    else
        return true;
}

bool PlanMerge::mergeCorridors(int left_idx, int right_idx,
        float coverage_threshold, float angular_threshold)
{
    if (corridors[left_idx].isSingleton() || corridors[right_idx].isSingleton())
        return false;
    if (!corridors[left_idx].bbox.intersects(corridors[right_idx].bbox))
        return false;

    mode = NONE;
    accumulator.clear();
    point_mapping.clear();
    merged_points.clear();
    size_t const original_corridor_size = corridors.size();

    bool did_merge = false;
    float cos_angular_threshold = cos(angular_threshold);
    //cerr << " angular limit: " << angular_threshold << " (cos=" << cos_angular_threshold << endl;

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
        cerr << left_p << endl;

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

        if (canMerge(*left_slice, *right_slice, distance, coverage_threshold, cos_angular_threshold))
        {
            did_merge = true;
            MedianPoint merged = *left_slice;
            PointID diff = right_p - left_p;
            merged.offset((right_p - left_p) / 2);

            pushMerged(left_idx, left_slice, right_idx, right_slice,
                    merged.center, merged);

            // Now, we're getting a little assymetry here ...
            //
            // We prefer 'left' to 'right' for merging. While we're going (probably) to
            // lose some data, we insert in merged_points and point_mapping
            // the neighbours of right_p that can also be merged with left_p. The goal
            // is that, when the pass on the remains of the right corridor is done, we
            // don't consider that some slices that have been skipped by the merging
            // pass cannot be merged.
            MedianLine::const_iterator other_it = right_slice;
            for (++other_it; other_it != right.median.end(); ++other_it)
            {
                if (merged_points.count(other_it->center))
                    break;
                if (canMerge(*left_slice, *other_it, distance, coverage_threshold, cos_angular_threshold))
                {
                    point_mapping[ make_pair(right_idx, other_it->center) ] =
                        make_pair(corridors.size(), merged.center);
                    merged_points.insert(other_it->center);
                } else break;
            }

            other_it = right_slice;
            for (--other_it; other_it != right.median.begin(); --other_it)
            {
                if (merged_points.count(other_it->center))
                    break;
                if (canMerge(*left_slice, *other_it, distance, coverage_threshold, cos_angular_threshold))
                {
                    point_mapping[ make_pair(right_idx, other_it->center) ] =
                        make_pair(corridors.size(), merged.center);
                    merged_points.insert(other_it->center);
                } else break;
            }
        }
        else
        {
            pushSingle(left_idx, left_slice);
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
    MedianLine::const_iterator last_slice = right.median.end();

    // These two indexes are the ones of the first and last corridors to
    // actually have points that were originally owned by the right side.
    //
    // This is used to initialize end_regions, end_types and to move the
    // connections
    int right_first_corridor = -1, right_last_corridor = -1;
    for (MedianLine::const_iterator right_slice = right.median.begin(); right_slice != right.median.end(); ++right_slice)
    {
        if (merged_points.count(right_slice->center))
        {
            cerr << right_slice->center << " is already merged" << endl;
            if (last_slice == right.median.end())
            {
                PtMapping::const_iterator map_it
                    = point_mapping.find( make_pair(right_idx, right_slice->center) );
                if (map_it == point_mapping.end())
                {
                    cerr << "no point mapping found for first slice: " << corridors[right_idx].name << " " << right_slice->center << endl;
                    throw runtime_error("cannot find a point mapping");
                }
                pair<int, PointID> mapped = map_it->second;
                right_first_corridor = mapped.first;
            }

            if (mode == SINGLE)
            {
                pushAccumulator();

                PtMapping::const_iterator map_it
                    = point_mapping.find( make_pair(right_idx, right_slice->center) );
                if (map_it == point_mapping.end())
                {
                    cerr << "no point mapping found for " << corridors[right_idx].name << " " << right_slice->center << endl;
                    throw runtime_error("cannot find a point mapping");
                }

                pair<int, PointID> mapped = map_it->second;

                Corridor& right_corridor = corridors.back();
                right_corridor.addConnection(last_slice->center, mapped.first, mapped.second);

                right_last_corridor = mapped.first;
                mode = NONE;

                cerr << "new right->merge connection " << right_corridor.name << " => " <<
                    corridors[mapped.first].name << endl;
            }
        }
        else
        {
            if (last_slice == right.median.end())
                right_first_corridor = corridors.size();
            right_last_corridor = corridors.size();

            // If we are getting out of a merged section, create the proper
            // connections. The only case where last_slice == end is for the
            // first RIGHT corridor
            if (mode == NONE && last_slice != right.median.end())
            {
                PtMapping::const_iterator map_it
                    = point_mapping.find( make_pair(right_idx, last_slice->center) );
                if (map_it == point_mapping.end())
                {
                    cerr << "no point mapping found for " << corridors[right_idx].name << " " << last_slice->center << endl;
                    throw runtime_error("cannot find a point mapping");
                }

                pair<int, PointID> mapped = map_it->second;

                Corridor& merged_corridor = corridors[mapped.first];
                merged_corridor.addConnection(map_it->second.second, corridors.size() - 1, right_slice->center);

                cerr << "new merge->right connection from " << merged_corridor.name << endl;
            }

            pushSingle(right_idx, right_slice);
            mode = SINGLE;
        }

        last_slice = right_slice;
    }

    // Push the leftovers from the pass on RIGHT slices
    if (mode == SINGLE)
    {
        pushAccumulator();
        right_last_corridor = corridors.size();
    }

    finalizeMerge(left_idx, right_idx, original_corridor_size,
            right_first_corridor, right_last_corridor);

    return true;
}

void PlanMerge::pushAccumulator()
{
    if (mode == MERGING)
        ownership.push_back(MERGED);
    else
        ownership.push_back(current_owner);

    corridors.push_back(accumulator);
    accumulator.clear();
}

void PlanMerge::pushSingle(int source_idx, MedianLine::const_iterator point)
{
    PointID median_point = point->center;
    //cerr << "NOT MERGING" << endl << endl;
    if (mode == MERGING)
        pushAccumulator();

    PtMappingTuple new_mapping = make_tuple(source_idx, median_point, corridors.size(), median_point);

    if (mode != SINGLE)
        cerr << "starting " << (current_owner == LEFT_SIDE ? "left" : "right") << " corridor at " << point->center << endl;
    if (mode != SINGLE)
        accumulator.name = corridors[source_idx].name + "." + boost::lexical_cast<std::string>(corridors.size());

    mode = SINGLE;

    // In point_mapping, we gather the mapping before the original point (in the
    // original corridor) and the point added to the accumulator. Both are, of
    // course, the same here but both points are different in case of a merge.
    point_mapping[ make_pair(source_idx, point->center) ] =
        make_pair(corridors.size(), point->center);
    accumulator.add(*point, true);
    last_point[0] = new_mapping;
}

void PlanMerge::pushMerged(
        int left_idx,  MedianLine::const_iterator left_p,
        int right_idx, MedianLine::const_iterator right_p,
        PointID const& p, MedianPoint const& median)
{
    PointID left_point  = left_p->center;
    PointID right_point = right_p->center;

    if (mode == SINGLE)
        pushAccumulator();

    int new_idx = corridors.size();
    PtMappingTuple from_left  = make_tuple(left_idx,  left_point,  corridors.size(), p);
    PtMappingTuple from_right = make_tuple(right_idx, right_point, corridors.size(), p);

    if (mode != MERGING)
        cerr << "starting to merge at " << left_p->center << " (" << right_p->center << ")" << endl;

    if (mode != MERGING)
        accumulator.name = "(" + corridors[left_idx].name + "|" + corridors[right_idx].name + ")." + boost::lexical_cast<std::string>(corridors.size());

    mode = MERGING;
    accumulator.add(p, median, true);

    last_point[0] = from_left;
    point_mapping[ make_pair(left_idx, left_point) ]   = make_pair(new_idx, p);
    last_point[1] = from_right;
    merged_points.insert(right_point);
    point_mapping[ make_pair(right_idx, right_point) ] = make_pair(new_idx, p);
}

void PlanMerge::finalizeMerge(size_t orig_left_idx, size_t orig_right_idx, size_t start_idx,
        size_t right_first_corridor, size_t right_last_corridor)
{
    Corridor const& orig_left  = corridors[orig_left_idx];
    Corridor const& orig_right = corridors[orig_right_idx];

    // The layout of the corridors is as follows:
    //  - first, we have a sequence of LEFT, MERGE, LEFT, MERGE, ... corridors.
    //    This sequence is done in the same direction than the left corridor
    //  - then, we get the set of RIGHT corridors. These corridors are oriented
    //    in the same direction than the right corridor, which can be different
    //    than the left corridor of course.
    //
    // During mergeCorridors(), the connections from merge to right are created,
    // as they would be too hard to create here. The rest has to be done here
    // ...

    // We treat the LEFT -> MERGE sequence first. We create connections, which
    // are either directed and in the same order than the corridors or
    // bidirectional if the left corridor is bidirectional.
    size_t left_sequence_end = find(ownership.begin() + start_idx, ownership.end(), RIGHT_SIDE) - ownership.begin();

    bool left_is_bidir  = orig_left.bidirectional;
    bool right_is_bidir = orig_right.bidirectional;
    bool merge_is_bidir = left_is_bidir || right_is_bidir;
    for (size_t i = start_idx; i < left_sequence_end; ++i)
    {
        Corridor& corridor = corridors[i];
        if (ownership[i] == MERGED)
            corridor.bidirectional = merge_is_bidir;
        else
            corridor.bidirectional = left_is_bidir;

        if (i > start_idx)
            corridor.end_regions[0].insert(corridor.median.front().center);
        if (i < left_sequence_end - 1)
        {
            corridor.end_regions[1].insert(corridor.median.back().center);
            corridor.addConnection(corridor.median.back().center, i + 1, corridors[i + 1].median.back().center);
        }

        if (i < left_sequence_end - 1)
        {
           if (!corridor.isConnectedTo(i + 1))
               cerr << "problem in left<->merge sequence: " << corridors[i] << " is not connected to " << corridors[i + 1] << endl;
           else if (ownership[i] == ownership[i + 1])
               cerr << "problem in left<->merge sequence: " << corridors[i] << " and " << corridors[i + 1] << " have the same ownership" << endl;
        }

        if (corridor.bidirectional)
        {
            corridor.end_types[0] = ENDPOINT_BIDIR;
            corridor.end_types[1] = ENDPOINT_BIDIR;
            if (i > start_idx)
                corridor.addConnection(corridor.median.front().center, i - 1, corridors[i - 1].median.back().center);
        }
        else
        {
            corridor.end_types[0] = ENDPOINT_BACK;
            corridor.end_types[1] = ENDPOINT_FRONT;
        }
    }

    // Then we manage the RIGHT corridors. Right now, we should only have
    // connections from right to merged
    size_t right_idx = right_first_corridor;
    int right_in_side = -1, right_out_side = -1;
    vector<bool> seen;
    seen.resize(corridors.size(), false);

    while (right_idx != right_last_corridor)
    {
        Corridor& corridor = corridors[right_idx];
        seen[right_idx] = true;

        int target_idx = -1;
        Corridor::Connections::iterator conn_it;
        for (conn_it = corridor.connections.begin(); conn_it != corridor.connections.end(); ++conn_it)
        {
            target_idx = conn_it->get<1>();
            if (target_idx < start_idx)
            {
                cerr << "in-merge corridor " << corridors[right_idx].name << " is connected to out-merge corridor " << corridors[target_idx].name << endl;
                throw runtime_error("found connection outside of newly generated corridors");
            }
            if (ownership[target_idx] == LEFT_SIDE)
            {
                if (ownership[right_idx] == RIGHT_SIDE)
                    throw runtime_error("right corridor connected to left");
                continue;
            }

            break;
        }

        if (conn_it == corridor.connections.end())
            throw runtime_error("error in right sequence: cannot reach end corridor");

        Corridor& target = corridors[target_idx];

        // For the first corridor, we obviously cannot use the inbound
        // connection, so we have to use a special case and check the outbound
        // connection.
        if (right_idx == right_first_corridor)
        {
            int in_side = corridor.findSideOf(conn_it->get<0>());
            right_in_side = !in_side;

            if (!corridor.bidirectional && ownership[right_idx] == MERGED)
            {
                if (in_side == 0)
                    corridor.bidirectional = true;
            }
        }
        if (target_idx == right_last_corridor)
            right_out_side = target.findSideOf(conn_it->get<2>());

        if (!target.bidirectional && ownership[target_idx] == MERGED)
        { 
            // merged corridors are not bidirectional ... or maybe they are.
            // Check that left and right are traversing the merged corridors in
            // the same direction.
            int target_side = target.findSideOf(conn_it->get<2>());
            if (target_side == 1)
                target.bidirectional = true;
        }

        if (ownership[right_idx] == RIGHT_SIDE)
            corridor.bidirectional = right_is_bidir;

        if (right_is_bidir)
            target.addConnection( conn_it->get<2>(), right_idx, conn_it->get<0>() );
    }

    if (right_out_side == -1)
        right_out_side = right_in_side;

    for (size_t corridor_idx = left_sequence_end; corridor_idx < corridors.size(); ++corridor_idx)
    {
        if (!seen[corridor_idx])
            cerr << "error in right sequence: " << corridors[corridor_idx].name << " is not included" << endl;
    }

    for (size_t corridor_idx = start_idx; corridor_idx < corridors.size(); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        if (corridor.bidirectional)
        {
            corridor.end_types[0] = ENDPOINT_BIDIR;
            corridor.end_types[1] = ENDPOINT_BIDIR;
        }
        else
        {
            corridor.end_types[0] = ENDPOINT_BACK;
            corridor.end_types[1] = ENDPOINT_FRONT;
        }
    }
    
    // Copy the necessary info from the original corridors
    corridors[start_idx].end_regions[0]       = orig_left.end_regions[0];
    corridors[left_sequence_end - 1].end_regions[1] = orig_left.end_regions[1];
    corridors[right_first_corridor].end_regions[right_in_side].insert(
            orig_right.end_regions[0].begin(), orig_right.end_regions[0].end() );
    corridors[right_last_corridor].end_regions[right_out_side].insert(
            orig_right.end_regions[1].begin(), orig_right.end_regions[1].end() );

    // Copy the outbound connections
    Corridor::Connections::const_iterator conn_it;
    for (conn_it = orig_left.connections.begin(); conn_it != orig_left.connections.end(); ++conn_it)
    {
        pair<int, PointID> mapping = 
            point_mapping[ make_pair(orig_left_idx, conn_it->get<0>()) ];
        corridors[mapping.first].addConnection( mapping.second, conn_it->get<1>(), conn_it->get<2>() );
    }
    for (conn_it = orig_right.connections.begin(); conn_it != orig_right.connections.end(); ++conn_it)
    {
        pair<int, PointID> mapping = 
            point_mapping[ make_pair(orig_right_idx, conn_it->get<0>()) ];
        corridors[mapping.first].addConnection( mapping.second, conn_it->get<1>(), conn_it->get<2>() );
    }

    // And finally update the inbound connections
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        Corridor::Connections& conn = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = conn.begin(); conn_it != conn.end(); ++conn_it)
        {
            int target_idx = conn_it->get<1>();
            if (target_idx != orig_left_idx && target_idx != orig_right_idx)
                continue;

            pair<int, PointID> mapping = 
                point_mapping[ make_pair(target_idx, conn_it->get<2>()) ];
            conn_it->get<1>() = mapping.first;
            conn_it->get<2>() = mapping.second;
        }
    }
    point_mapping.clear();

}

//void PlanMerge::copyConnections()
//{
//    cerr << "merge: copying connections" << endl;
//
//    for (size_t i = 0; i < corridors.size(); ++i)
//    {
//        Corridor::Connections& conn = corridors[i].connections;
//        Corridor::Connections::iterator conn_it;
//        for (conn_it = conn.begin(); conn_it != conn.end(); ++conn_it)
//        {
//            PtMapping::const_iterator src_map = point_mapping.find( make_pair(i, conn_it->get<0>()) );
//            if (src_map == point_mapping.end())
//            {
//                if (ownership[i] == TO_DELETE)
//                {
//                    cerr << "a connection cannot be copied: " << i << " " << conn_it->get<0>() << endl;
//                    //throw std::runtime_error("a connection cannot be copied");
//                }
//
//                PtMapping::const_iterator dst_map = point_mapping.find( make_pair(conn_it->get<1>(), conn_it->get<2>()) );
//                if (dst_map != point_mapping.end())
//                {
//                    cerr << "  updating target: "
//                        << corridors[i].name << " " << conn_it->get<0>() << " -> "
//                        << corridors[conn_it->get<1>()].name << " " << conn_it->get<2>()
//                        << " moved to " << corridors[dst_map->second.first].name << " " << dst_map->second.second
//                        << endl;
//
//                    conn_it->get<1>() = dst_map->second.first;
//                    conn_it->get<2>() = dst_map->second.second;
//                }
//            }
//            else
//            {
//                int new_idx = src_map->second.first;
//                Corridor::Connections& new_conn = corridors[new_idx].connections;
//                PointID   new_point = src_map->second.second;
//                
//                PtMapping::const_iterator dst_map = point_mapping.find( make_pair(conn_it->get<1>(), conn_it->get<2>()) );
//                if (dst_map == point_mapping.end())
//                {
//                    cerr << "  new connection: "
//                        << corridors[new_idx].name << " " << new_point << " -> "
//                        << corridors[conn_it->get<1>()].name << " " << conn_it->get<2>() << endl;
//
//                    new_conn.push_back( make_tuple(new_point, conn_it->get<1>(), conn_it->get<2>()) );
//                }
//                else
//                {
//                    cerr << "  new connection: "
//                        << corridors[new_idx].name << " " << new_point << " -> "
//                        << corridors[dst_map->second.first].name << " " << dst_map->second.second << endl;
//
//                    new_conn.push_back( make_tuple(new_point, dst_map->second.first, dst_map->second.second) );
//                }
//            }
//        }
//    }
//    point_mapping.clear();
//}

