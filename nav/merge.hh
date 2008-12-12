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

        void pushPoint(int target_idx, Corridor& target, Corridor const& source, MedianLine::const_iterator median);

        void mergeCorridors(Corridor const& left, Corridor const& right, std::vector<Corridor>& result,
                float coverage_threshold, float angular_threshold);
        void pushLeftRight(Corridor const& left, MedianLine::const_iterator left_p,
                Corridor const& right, MedianLine::const_iterator right_p);
        void pushMerged(Corridor const& left, MedianLine::const_iterator left_p,
                Corridor const& right, MedianLine::const_iterator right_p,
                PointID const& p, MedianPoint const& median);

        void finalizeMerge();

    public:
        PlanMerge();
        void merge(Plan const& left, Plan const& right);
    };
}

#endif

