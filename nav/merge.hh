#ifndef NAV_MERGE_HH
#define NAV_MERGE_HH

#include "plan.hh"

namespace Nav
{
    class PlanMerge : public Plan
    {
        Corridor left_corridor, right_corridor;
        Corridor merged_corridor;

        enum MERGING_MODES
        { NONE, LEFT_RIGHT, MERGING };
        MERGING_MODES mode;

        void copyConnections(int target_idx, Corridor& target, PointID const& target_p,
                Corridor const& source, PointID const& source_p);

        void pushLeftRight(Corridor const& left, MedianLine::const_iterator left_p,
                Corridor const& right, MedianLine::const_iterator right_p);
        void pushMerged(Corridor const& left, MedianLine::const_iterator left_p,
                Corridor const& right, MedianLine::const_iterator right_p,
                PointID const& p, MedianPoint const& median);

        void finalizeMerge();

    public:
        PlanMerge();
        void merge(Plan const& left, Plan const& right);
        bool mergeCorridors(int left_idx, int right_idx,
                float coverage_threshold, float angular_threshold);
    };
}

#endif

