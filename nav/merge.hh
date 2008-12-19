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

        typedef std::list< boost::tuple<PointID, int, bool> > EndPoints;
        EndPoints endpoints;

        /** This type and this list hold the set of mapping from the given
         * corridor into the merged ones. The tuple is
         *  ( source_point, source_corridor, new_point )
         */
        typedef boost::tuple<PointID, Corridor const*, PointID> AccMappingTuple;
        std::list<AccMappingTuple> accumulator_point_mapping;

        typedef boost::tuple<PointID, int, PointID> PtMappingTuple;
        std::list<PtMappingTuple> point_mapping;

        PointSet merged_points;

        enum MERGING_MODES
        { NONE, SINGLE, MERGING };
        MERGING_MODES mode;

        void copyConnections(int target_idx, Corridor& target, PointID const& target_p,
                Corridor const& source, PointID const& source_p);

        void pushAccumulator();
        void pushSingle(Corridor const& corridor, MedianLine::const_iterator point);
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

