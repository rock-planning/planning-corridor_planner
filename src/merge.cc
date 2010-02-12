#include "merge.hh"
#include <cmath>
#include <stdexcept>
#include <iostream>

#include <algorithm>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

using namespace nav;
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

    //for (int corr_idx = 0; corr_idx < corridors.size(); ++corr_idx)
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
    for (int i = 0; i < static_cast<int>(corridors.size()); ++i)
    {
        Corridor const& corridor = corridors[i];
        if (corridor.isSingleton())
        {
            PointID const p = corridor.front().center;
            if ((p == left.getStartPoint() || p == right.getStartPoint()
                 || p == left.getEndPoint() || p == right.getEndPoint()))
            {
                cerr << "corridor " << i << " is an endpoint corridor" << endl;
                useful_corridors[i] = USEFUL;
            }
        }
    }
    //markUselessCorridors(useful_corridors);
    //removeUselessCorridors(useful_corridors);
    //mergeSimpleCrossroads_directed();
}

void PlanMerge::process(float coverage_threshold, float angular_threshold)
{
    for (int i = 0; i < static_cast<int>(corridors.size()); ++i)
    {
        if (ownership[i] == TO_DELETE || ownership[i] == MERGED)
            continue;

        cerr << "looking at corridor " << corridors[i].name << endl;

        int original_corridors_size = corridors.size();
        bool did_merge = false;
        int j;
        for (j = 0; j < static_cast<int>(corridors.size()); ++j)
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
            for (int new_idx = original_corridors_size; new_idx < static_cast<int>(corridors.size()); ++new_idx)
                cerr << " " << corridors[new_idx].name;
            cerr << " ]" << endl;

            // A merge happened, remove the original corridors and adjust the
            // indexes
            ownership[i] = ownership[j] = TO_DELETE;
        }
    }

    for (int i = 0, owner_i = 0; i < static_cast<int>(corridors.size()); ++owner_i)
    {
        if (ownership[owner_i] == TO_DELETE)
            removeCorridor(i);
        else
            ++i;
    }
    mergeSimpleCrossroads_directed();
}

//pair<false,  PlanMerge::findMergeCandidate(MedianPoint const& left_slice, Corridor const& right,
//        float coverage_threshold, float cos_angular_threshold)
//{
//    PointID left_p = left_slice.center;
//
//    Corridor::voronoi_const_iterator min_slice   = corridor.median.begin();
//    Corridor::voronoi_const_iterator right_slice = min_slice;
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
//        return make_pair(false, Corridor::voronoi_const_iterator());
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
//        return make_pair(false, Corridor::voronoi_const_iterator());
//
//    // OK, the two slices are more or less parallel. Check that they also
//    // intersect
//    float left_w = left_slice->width;
//    float right_w = right_slice->width;
//
//    if ((right_w - (left_w - para_distance)) / right_w > 1 - coverage_threshold)
//        return make_pair(false, Corridor::voronoi_const_iterator());
//    else if ((left_w - (right_w - para_distance)) / left_w > 1 - coverage_threshold)
//        return make_pair(false, Corridor::voronoi_const_iterator());
//    else
//        return make_pair(true, right_slice);
//}

bool PlanMerge::canMerge(VoronoiPoint const& left_p, VoronoiPoint const& right_p,
        float coverage_threshold, float cos_angular_threshold) const
{
    return canMerge(left_p, right_p, left_p.center.distance(right_p.center), coverage_threshold, cos_angular_threshold);
}

bool PlanMerge::canMerge(VoronoiPoint const& left_slice, VoronoiPoint const& right_slice,
        float distance, float coverage_threshold, float cos_angular_threshold) const
{
    // Now, check the principal direction of both the points
    Point<float> left_tg  = left_slice.tangent;
    Point<float> right_tg = right_slice.tangent;

    PointID left_p  = left_slice.center;
    PointID right_p = right_slice.center;

    float cos_angle = fabs(left_tg * right_tg);
    //cerr << left_p << " " << right_p << " left_dir=" << left_dir << " right_dir=" << right_dir << "\n" << " angle=" << cos_angle << "\n";

    if (cos_angle < cos_angular_threshold)
        return false;

    // Now, check two things
    //  - that the two corridors are near to each other (perpendicularly to
    //    the corridor main direction)
    //  - that the two corridors overlap (along the corridor main direction)
    Point<float> dist_dir = left_p - right_p;
    float ortho_distance = fabs(dist_dir * left_tg);
    float para_distance = fabs(sqrt(distance * distance - ortho_distance * ortho_distance));

    //cerr << " d=" << distance << " para_distance=" << para_distance << " ortho_distance=" << ortho_distance << endl;

    if (ortho_distance > 4)
        return false;

    // OK, the two slices are more or less parallel. Check that they also
    // intersect
    float w0 = left_slice.width;
    float w1 = right_slice.width;

    if (w1 > w0)
        swap(w0, w1);

    float diff = max<float>(para_distance + w1 - w0, 0);
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
    int const original_corridor_size = corridors.size();

    bool did_merge = false;
    float cos_angular_threshold = cos(angular_threshold);
    //cerr << " angular limit: " << angular_threshold << " (cos=" << cos_angular_threshold << endl;

    // There is a copy here because of an initial (bad) decision to refer to
    // other corridors using indexes. Therefore, when we add new corridors to
    // vector<>, left and right may get invalidated, fucking up the loop
    Corridor left  = corridors[left_idx];
    Corridor right = corridors[right_idx];

    // First, traverse the left corridor and build left-only and merged
    // corridors
    current_owner = LEFT_SIDE; // used by pushAccumulator to mark the ownership of new corridors
    for (list<VoronoiPoint>::const_iterator left_slice = left.voronoi.begin(); left_slice != left.voronoi.end(); ++left_slice)
    {
        PointID left_p = left_slice->center;

        // Find the median point in +right+ that is closest to left_p
        Corridor::voronoi_const_iterator right_slice = right.voronoi.end();
        float distance = -1;
        Corridor::voronoi_const_iterator min_slice;
        for (min_slice = right.voronoi.begin(); min_slice != right.voronoi.end(); ++min_slice)
        {
            PointID p = min_slice->center;
            float d   = left_p.distance(p);
            if (!canMerge(*left_slice, *min_slice, d, coverage_threshold, cos_angular_threshold))
                continue;

            if (right_slice == right.voronoi.end() || d < distance)
            {
                right_slice = min_slice;
                distance = d;
            }
        }

        if (right_slice != right.voronoi.end())
        {
            did_merge = true;
            VoronoiPoint merged = *left_slice;
            //PointID diff = right_p - left_p;
            //merged.offset((right_p - left_p) / 2);

            cerr << "adding " << merged.center << " to merge " << left_slice->center << " and " << right_slice->center << endl;
            pushMerged(left_idx, left_slice, right_idx, right_slice,
                    merged.center, merged);
        }
        else
        {
            cerr << "adding " << left_slice->center << " to left" << endl;
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
    Corridor::voronoi_const_iterator last_slice = right.voronoi.end();

    // These two indexes are the ones of the first and last corridors to
    // actually have points that were originally owned by the right side.
    //
    // This is used to initialize end_regions, end_types and to move the
    // connections
    int right_first_corridor = -1, right_last_corridor = -1;
    Corridor::voronoi_const_iterator last_merge = right.voronoi.end(),
        next_merge = right.voronoi.begin();
    for (Corridor::voronoi_const_iterator right_slice = right.voronoi.begin(); right_slice != right.voronoi.end(); ++right_slice)
    {
        bool merged = merged_points.count(right_slice->center);
        if (merged)
        {
            // DO NOT update last_merge here. The right last_merge value is
            // needed in the merge part
            //
            // we use begin() to mark not-initialized and end() if there is no next merge
            next_merge = right.voronoi.begin(); 
        }
        else
        {
            // Check that this point should actually not be merged. We do that
            // by looking at the last point that has been merged (if there is
            // one) and/or the next point that is merged.
            if (next_merge == right.voronoi.begin())
            {
                for (next_merge = right_slice;
                        !merged_points.count(next_merge->center) && next_merge != right.voronoi.end();
                        ++next_merge);
            }

            pair<int, PointID> mapping;
            if (last_merge != right.voronoi.end() || next_merge != right.voronoi.end())
            {
                if (last_merge != right.voronoi.end())
                {
                    mapping = point_mapping[ make_pair(right_idx, last_merge->center) ];
                    Corridor::voronoi_const_iterator median_it =
                        corridors[mapping.first].findMedianPoint(mapping.second);
                    merged = canMerge(*median_it, *right_slice, coverage_threshold, cos_angular_threshold);
                }

                if (!merged && next_merge != right.voronoi.end())
                {
                    mapping = point_mapping[ make_pair(right_idx, next_merge->center) ];
                    Corridor::voronoi_const_iterator median_it =
                        corridors[mapping.first].findMedianPoint(mapping.second);
                    merged = canMerge(*median_it, *right_slice, coverage_threshold, cos_angular_threshold);
                }
            }

            if (merged)
            {
                point_mapping[ make_pair(right_idx, right_slice->center) ] = mapping;

                cerr << "  using " << mapping.second << " as merge point for " << right_slice->center << endl;
                merged_points.insert(right_slice->center);
            }
        }
        
        if (merged)
        {
            // if this is the first slice, we need to initialize right_first_corridor and right_last_corridor
            if (last_slice == right.voronoi.end())
            {
                PtMapping::const_iterator map_it
                    = point_mapping.find( make_pair(right_idx, right_slice->center) );
                if (map_it == point_mapping.end())
                {
                    cerr << "no point mapping found for first slice: " << corridors[right_idx].name << " " << right_slice->center << endl;
                    throw runtime_error("cannot find a point mapping");
                }
                pair<int, PointID> mapped = map_it->second;
                right_first_corridor = right_last_corridor = mapped.first;
            }

            // Then get the mapping for the point and update some tracking
            // variables
            PtMapping::const_iterator map_it
                = point_mapping.find( make_pair(right_idx, right_slice->center) );
            if (map_it == point_mapping.end())
            {
                cerr << "no point mapping found for " << corridors[right_idx].name << " " << right_slice->center << endl;
                throw runtime_error("cannot find a point mapping");
            }

            pair<int, PointID> mapped = map_it->second;
            if (mode == NONE && last_slice != right.voronoi.end() && mapped.first != right_last_corridor)
            {
                cerr << "  right side is bridging " << corridors[right_last_corridor].name << " => " << corridors[mapped.first].name << ", adding the necessary connection" << endl;
                pair<int, PointID> last_mapping = point_mapping[ make_pair(right_idx, last_merge->center) ];
                corridors[right_last_corridor].addConnection(
                        last_mapping.second, mapped.first, mapped.second);
                if (right_last_corridor + 2 == mapped.first && ownership[right_last_corridor + 1] != TO_DELETE)
                {
                    Corridor& merge = corridors[right_last_corridor];
                    Corridor& cancelled = corridors[right_last_corridor + 1];
                    cerr << "  cancelling useless left corridor " << cancelled.name << endl;

                    for (Corridor::voronoi_const_iterator it = cancelled.begin(); it != cancelled.end(); ++it)
                    {
                        point_mapping[ make_pair(right_last_corridor + 1, it->center) ].first = right_last_corridor;
                        merge.push_back(*it);
                    }

                    ownership[right_last_corridor + 1] = TO_DELETE;
                }
            }
            else if (mode == SINGLE)
            {
                // Check first that we're not looping on the same merge corridor
                if (corridors[mapped.first].isConnectedTo(corridors.size()))
                {
                    cerr << "going back into the same merge corridor, cancelling " << accumulator.name << endl;
                    corridors[mapped.first].removeConnectionsTo(corridors.size());
                    accumulator.clear();
                }
                else
                {
                    pushAccumulator();
                    Corridor& right_corridor = corridors.back();
                    right_corridor.addConnection(last_slice->center, mapped.first, mapped.second);
                    cerr << "  new right->merge connection " << right_corridor.name << " => " <<
                        corridors[mapped.first].name << " " << mapped.second << endl;
                }

                mode = NONE;
            }

            last_merge = right_slice;
            right_last_corridor = mapped.first;
            cerr << "  " << right_slice->center << " is already merged in " << corridors[mapped.first].name << " as " << mapped.second << endl;
        }
        else
        {
            // If we are getting out of a merged section, create the proper
            // connections. The only case where last_slice == end is for the
            // first RIGHT corridor
            if (mode == NONE && last_slice != right.voronoi.end())
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
                merged_corridor.addConnection(mapped.second, corridors.size(), right_slice->center);

                cerr << "new merge->right connection from " << merged_corridor.name <<
                    mapped.second << " to " << right_slice->center << endl;
            }

            cerr << "adding " << right_slice->center << " to right" << endl;
            pushSingle(right_idx, right_slice);
            mode = SINGLE;

            if (last_slice == right.voronoi.end())
                right_first_corridor = corridors.size();
            right_last_corridor = corridors.size();
        }

        last_slice = right_slice;
    }

    // Push the leftovers from the pass on RIGHT slices
    if (mode == SINGLE)
    {
        pushAccumulator();
        right_last_corridor = corridors.size() - 1;
    }

    cerr << "right side starts at " << right_first_corridor << " and finishes at " << right_last_corridor << endl;
    cerr << "right side starts at " << corridors[right_first_corridor].name << " and finishes at " << corridors[right_last_corridor].name << endl;

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
    cerr << "created " << corridors.back().name << endl;
    accumulator.clear();
}

void PlanMerge::pushSingle(int source_idx, Corridor::voronoi_const_iterator point)
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
    accumulator.push_back(*point);
    last_point[0] = new_mapping;
}

void PlanMerge::pushMerged(
        int left_idx,  Corridor::voronoi_const_iterator left_p,
        int right_idx, Corridor::voronoi_const_iterator right_p,
        PointID const& p, VoronoiPoint const& voronoi)
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
        accumulator.name = "(" + corridors[left_idx].name + "|" + corridors[right_idx].name + ")" + "." + boost::lexical_cast<std::string>(corridors.size());

    mode = MERGING;
    accumulator.push_back(p, voronoi);

    last_point[0] = from_left;
    point_mapping[ make_pair(left_idx, left_point) ]   = make_pair(new_idx, p);
    last_point[1] = from_right;
    merged_points.insert(right_point);
    point_mapping[ make_pair(right_idx, right_point) ] = make_pair(new_idx, p);
}

pair<int, PointID> PlanMerge::getEndpointMapping(PointID const& p, Corridor const& orig, int first_idx, int last_idx) const
{
    int side = orig.findSideOf(p);
    if (side == ENDPOINT_BACK)
        return make_pair(last_idx, corridors[last_idx].back().center);
    else
        return make_pair(first_idx, corridors[first_idx].front().center);
}

void PlanMerge::finalizeMerge(int orig_left_idx, int orig_right_idx, int start_idx,
        int right_first_corridor, int right_last_corridor)
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
    int left_sequence_end = find(ownership.begin() + start_idx, ownership.end(), RIGHT_SIDE) - ownership.begin();
    int left_first_corridor = -1, left_last_corridor = -1;

    bool left_is_bidir  = orig_left.bidirectional;
    bool right_is_bidir = orig_right.bidirectional;
    bool merge_is_bidir = left_is_bidir || right_is_bidir;
    int previous_left_corridor = -1;
    for (int i = start_idx; i < left_sequence_end; ++i)
    {
        Corridor& corridor = corridors[i];

        // This corridor has been marked as bridged by the RIGHT pass of
        // mergeCorridors. Ignore it.
        if (ownership[i] == TO_DELETE)
        {
            cerr << "  left corridor " << corridor.name << " is already cancelled" << endl;
            continue;
        }

        if (corridor.size() < 4 && ownership[i] == LEFT_SIDE)
        {
            cerr << "  cancelling small corridor " << corridor.name << endl;
            ownership[i] = TO_DELETE;
            continue;
        }

        if (ownership[i] == MERGED && previous_left_corridor != -1)
        {
            Corridor& prev = corridors[previous_left_corridor];
            if (ownership[previous_left_corridor] == MERGED)
            {
                cerr << "  merging " << corridor.name << " into " << prev.name << endl;
                corridor.bidirectional = prev.bidirectional; // mark the bidirectional flag to make Corridor::merge happy
                prev.merge(corridor);
                prev.end_regions[1].clear();
                prev.end_regions[1].insert(prev.back().center);

                // Copy the outbound connections (to right corridors)
                prev.connections.insert( prev.connections.end(), corridor.connections.begin(), corridor.connections.end() );
                // ... then fix the inbound connections from the right corridors
                // as well
                for (int right_idx = left_sequence_end; right_idx < static_cast<int>(corridors.size()); ++right_idx)
                {
                    if (prev.isConnectedTo(right_idx) && corridors[right_idx].isConnectedTo(i))
                        prev.removeConnectionsTo(right_idx);
                    else
                        corridors[right_idx].moveConnections(i, previous_left_corridor);
                }
                if (right_last_corridor == i)
                    right_last_corridor = previous_left_corridor;

                ownership[i] = TO_DELETE;
                continue;
            }
        }

        if (left_first_corridor == -1)
            left_first_corridor = i;
        left_last_corridor = i;

        if (ownership[i] == MERGED)
            corridor.bidirectional = merge_is_bidir;
        else
            corridor.bidirectional = left_is_bidir;

        corridor.end_regions[0].insert(corridor.front().center);
        corridor.end_regions[1].insert(corridor.back().center);
        if (previous_left_corridor != -1)
        {
            cerr << "  connecting " << corridors[previous_left_corridor].name << " to " << corridors[i].name << endl;
            corridors[previous_left_corridor].addConnection(corridors[previous_left_corridor].backPoint(), i, corridors[i].frontPoint());
        }
        // Always connect from the previous item in the sequence (if this is not
        // the first)
        //if (previous_left_corridor != start_idx)
        //    corridor.addConnection(corridors[previous_left_corridor].median.back().center, i, corridor.median.front().center);

        if (previous_left_corridor != -1)
        {
           if (!corridors[previous_left_corridor].isConnectedTo(i))
               cerr << "problem in left<->merge sequence: " << corridors[previous_left_corridor].name << " is not connected to " << corridors[i].name << endl;
           else if (ownership[i] == LEFT_SIDE && ownership[previous_left_corridor] == LEFT_SIDE)
               cerr << "problem in left<->merge sequence: " << corridors[i].name << " and " << corridors[i + 1].name << " are both owned by LEFT" << endl;
        }

        if (corridor.bidirectional)
        {
            if (previous_left_corridor != -1)
            {
                cerr << "  creating reverse connection (bidir): " << corridor.name << " => " << corridors[previous_left_corridor].name << endl;
                corridor.addConnection(corridor.frontPoint(), previous_left_corridor, corridors[previous_left_corridor].backPoint());
            }
            //if (previous_left_corridor != start_idx)
            //    corridor.addConnection(corridor.median.front().center, previous_left_corridor, corridors[previous_left_corridor].median.back().center);
        }
        previous_left_corridor = i;
    }

    // Then we manage the RIGHT corridors. Right now, we should only have
    // connections from right to merged
    for (int right_idx = left_sequence_end; right_idx < static_cast<int>(corridors.size()); ++right_idx)
    {
        Corridor& corridor = corridors[right_idx];
        corridor.end_regions[0].insert(corridor.frontPoint());
        corridor.end_regions[1].insert(corridor.backPoint());

        if (right_idx != right_first_corridor)
            corridor.end_types[0] = ENDPOINT_FRONT;
        if (right_idx != right_last_corridor)
            corridor.end_types[1] = ENDPOINT_BACK;
        if (right_is_bidir)
            corridor.bidirectional = right_is_bidir;
    }

    cerr << "fixing connections for right side" << endl;
    cerr << "  right side is starting at " << corridors[right_first_corridor].name << " (" << right_first_corridor << ")" << endl;
    cerr << "  right side is finishing at " << corridors[right_last_corridor].name <<" (" << right_last_corridor << ")" <<  endl;
    int previous_right_idx = -1;
    vector<bool> seen;
    seen.resize(corridors.size(), false);
    int right_idx = right_first_corridor;
    while (right_idx != right_last_corridor)
    {
        Corridor& corridor = corridors[right_idx];
        seen[right_idx] = true;

        int target_idx = -1;
        Corridor::Connections::iterator conn_it;
        for (conn_it = corridor.connections.begin(); conn_it != corridor.connections.end(); ++conn_it)
        {
            target_idx = conn_it->get<1>();
            if (seen[target_idx])
                continue;

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
        cerr << "    looking at " << corridor.name << ", next right corridor is " << target.name << endl;

        if (!target.bidirectional && ownership[target_idx] == MERGED)
        { 
            // merged corridors are not bidirectional ... or maybe they are.
            // Check that left and right are traversing the merged corridors in
            // the same direction.
            int target_side = target.findSideOf(conn_it->get<2>());
            if (target_side == ENDPOINT_BACK)
            {
                cerr << "  corridor " << corridor.name << " is traversed in both directions, marking as bidirectional" << endl;
                target.bidirectional = true;
            }
        }

        if (right_is_bidir)
        {
            cerr << "  adding connection " << target.name << " => " << corridor.name << " (bidir)" << endl;
            target.addConnection( conn_it->get<2>(), right_idx, conn_it->get<0>() );
        }

        // Cancel small corridors. Note that it HAS to be done AFTER we fix the
        // bidirectional flag for the connection target, otherwise that flag
        // won't get updated properly.
        if (ownership[right_idx] == RIGHT_SIDE && corridor.size() < 4)
        {
            if (previous_right_idx != -1)
            {
                corridors[previous_right_idx].moveConnections(right_idx, target_idx);
                corridors[previous_right_idx].connections.insert(
                        corridors[previous_right_idx].connections.begin(),
                        corridor.connections.begin(), corridor.connections.end());
            }

            if (right_idx == right_first_corridor)
                right_first_corridor = target_idx;
            // right_idx cannot be right_last_corridor

            ownership[right_idx] = TO_DELETE;
        }
        else
            previous_right_idx = right_idx;

        right_idx = target_idx;
    }
    seen[right_last_corridor] = true;
    if (ownership[right_last_corridor] == RIGHT_SIDE && corridors[right_last_corridor].size() < 4)
    {
        ownership[right_last_corridor] = TO_DELETE;
        right_last_corridor = previous_right_idx;
    }

    for (int corridor_idx = left_sequence_end; corridor_idx < static_cast<int>(corridors.size()); ++corridor_idx)
    {
        if (!seen[corridor_idx])
            cerr << "error in right sequence: " << corridors[corridor_idx].name << " is not included" << endl;
    }

    int right_in_side  = corridors[right_first_corridor].findSideOf(orig_right.frontPoint());
    int right_out_side = corridors[right_last_corridor].findSideOf(orig_right.backPoint());
    if (right_in_side == ENDPOINT_BACK)
        corridors[right_first_corridor].bidirectional = true;
    if (right_out_side == ENDPOINT_FRONT)
        corridors[right_last_corridor].bidirectional = true;

    for (int corridor_idx = start_idx; corridor_idx < static_cast<int>(corridors.size()); ++corridor_idx)
    {
        Corridor& corridor = corridors[corridor_idx];
        if (corridor.bidirectional)
        {
            corridor.end_types[0] = ENDPOINT_BIDIR;
            corridor.end_types[1] = ENDPOINT_BIDIR;
        }
        else
        {
            corridor.end_types[0] = ENDPOINT_FRONT;
            corridor.end_types[1] = ENDPOINT_BACK;
        }
    }

    // Copy the necessary info from the original corridors
    corridors[left_first_corridor].end_regions[0] = orig_left.end_regions[0];
    corridors[left_last_corridor].end_regions[1]  = orig_left.end_regions[1];

    corridors[right_first_corridor].end_regions[right_in_side].insert(
            orig_right.end_regions[0].begin(), orig_right.end_regions[0].end() );
    corridors[right_last_corridor].end_regions[right_out_side].insert(
            orig_right.end_regions[1].begin(), orig_right.end_regions[1].end() );

    // Copy the outbound connections
    Corridor::Connections::const_iterator conn_it;
    for (conn_it = orig_left.connections.begin(); conn_it != orig_left.connections.end(); ++conn_it)
    {
        pair<int, PointID> mapping = getEndpointMapping(conn_it->get<0>(), orig_left, left_first_corridor, left_last_corridor);
        cerr << "  copying connection " 
            << "(" << orig_left.name << ", " << conn_it->get<0>() << ", " << corridors[conn_it->get<1>()].name << ", " << conn_it->get<2>() << ")"
            << " to "
            << "(" << corridors[mapping.first].name << ", " << mapping.second << ", " << corridors[conn_it->get<1>()].name << ", " << conn_it->get<2>() << ")" << endl;
        corridors[mapping.first].addConnection( mapping.second, conn_it->get<1>(), conn_it->get<2>());
    }
    for (conn_it = orig_right.connections.begin(); conn_it != orig_right.connections.end(); ++conn_it)
    {
        pair<int, PointID> mapping = getEndpointMapping(conn_it->get<0>(), orig_right, right_first_corridor, right_last_corridor);
        cerr << "  copying connection " 
            << "(" << orig_right.name << ", " << conn_it->get<0>() << ", " << corridors[conn_it->get<1>()].name << ", " << conn_it->get<2>() << ")"
            << " to "
            << "(" << corridors[mapping.first].name << ", " << mapping.second << ", " << corridors[conn_it->get<1>()].name << ", " << conn_it->get<2>() << ")" << endl;
        corridors[mapping.first].addConnection( mapping.second, conn_it->get<1>(), conn_it->get<2>());
    }

    // And finally update the inbound connections
    for (int i = 0; i < start_idx; ++i)
    {
        Corridor::Connections& conn = corridors[i].connections;
        Corridor::Connections::iterator conn_it;
        for (conn_it = conn.begin(); conn_it != conn.end(); ++conn_it)
        {
            int target_idx = conn_it->get<1>();
            if (target_idx != orig_left_idx && target_idx != orig_right_idx)
                continue;

            pair<int, PointID> mapping;
            if (target_idx == orig_left_idx)
                mapping = getEndpointMapping(conn_it->get<2>(), orig_left, left_first_corridor, left_last_corridor);
            else
                mapping = getEndpointMapping(conn_it->get<2>(), orig_right, right_first_corridor, right_last_corridor);

            cerr << "  updating inbound connection "
                << "(" << corridors[i].name << ", " << conn_it->get<0>() << ", " << corridors[conn_it->get<1>()].name << ", " << conn_it->get<2>() << ")"
                << " to " << corridors[mapping.first].name << " " << mapping.second << endl;
            conn_it->get<1>() = mapping.first;
            conn_it->get<2>() = mapping.second;
        }
    }

    // Now, try to do a bit of cleanup. Namely, small intermediate left or right
    // corridors are removed.
    //for (int i = 0; i < corridors.size(); ++i)
    //{
    //    Corridor& corridor = corridors[i];
    //    Corridor::Connections& conn = corridors[i].connections;
    //    if (ownership[i] != MERGED)
    //        continue;

    //    Corridor::Connections::iterator conn_it = conn.begin();
    //    for (conn_it = conn.begin(); conn_it != conn.end(); ++conn_it)
    //    {
    //        int target_idx = conn_it->get<1>();
    //        if (target_idx < start_idx || ownership[target_idx] == MERGED)
    //            continue;

    //        if (target_idx >= corridors.size())
    //            cerr << "connection found to non-existing corridor " << target_idx << endl;

    //        Corridor& target = corridors[target_idx];
    //        if (target.median.size() > 4)
    //            continue;

    //        int target_side = corridor.findSideOf(conn_it->get<0>());
    //        PointID new_source;
    //        if (target_side == 0)
    //            new_source = corridor.median.back().center;
    //        else
    //            new_source = corridor.median.front().center;

    //        cerr << "  cancelling corridor " << target.name << endl;
    //        ownership[target_idx] = TO_DELETE;

    //        Corridor::Connections::iterator target_conn_it = target.connections.begin();
    //        for (; target_conn_it != target.connections.end(); ++target_conn_it)
    //            corridor.addConnection(new_source, target_conn_it->get<1>(), target_conn_it->get<2>());
    //    }
    //}

    point_mapping.clear();

}

//void PlanMerge::copyConnections()
//{
//    cerr << "merge: copying connections" << endl;
//
//    for (int i = 0; i < corridors.size(); ++i)
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

