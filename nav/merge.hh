#ifndef NAV_MERGE_HH
#define NAV_MERGE_HH

#include "plan.hh"

namespace Nav
{
    class PlanMerge : public Plan
    {
        /** Temporary corridor used during the merge process. New points are
         * accumulated in there. When required, this corridor is updated with
         * connection points and inserted in Plan::corridors by
         * pushAccumulator()
         */
        Corridor accumulator;

        typedef boost::tuple<int, PointID, int, PointID> PtMappingTuple;
        typedef std::map< std::pair<int, PointID>, std::pair<int, PointID> > PtMapping;
        PtMapping point_mapping;

        PtMappingTuple last_point[2];

        PointSet merged_points;

        enum MERGING_MODES
        { NONE, SINGLE, MERGING };
        MERGING_MODES mode;

        enum OWNERSHIP
        { TO_DELETE, LEFT_SIDE, RIGHT_SIDE, MERGED };
        std::vector<OWNERSHIP> ownership;
        OWNERSHIP current_owner;

        /** Returns true if +left_p+ and +right_p+ can be merged */
        bool canMerge(MedianPoint const& left_p, MedianPoint const& right_p,
                float coverage_threshold, float cos_angular_threshold) const;

        /** Returns true if +left_p+ and +right_p+ can be merged */
        bool canMerge(MedianPoint const& left_p, MedianPoint const& right_p,
                float distance, float coverage_threshold, float cos_angular_threshold) const;

        void finalizeMerge(size_t orig_left_idx, size_t orig_right_idx, size_t start_idx,
                size_t right_first_corridor, size_t right_last_corridor);

        void pushAccumulator();
        void pushSingle(int corridor_idx, MedianLine::const_iterator point);
        void pushMerged(int left_idx, MedianLine::const_iterator left_p,
                int right_idx, MedianLine::const_iterator right_p,
                PointID const& p, MedianPoint const& median);

    public:
        PlanMerge();
        void merge(Plan const& left, Plan const& right, float coverage_threshold, float angular_threshold);
        void process(float coverage_threshold, float angular_threshold);
        bool mergeCorridors(int left_idx, int right_idx,
                float coverage_threshold, float angular_threshold);
    };
}

#endif

