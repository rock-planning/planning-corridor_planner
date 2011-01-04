#ifndef NAV_DSTAR_HPP
#define NAV_DSTAR_HPP

#include <corridor_planner/point.hh>
#include <corridor_planner/pool_allocator.hh>
#include <corridor_planner/traversability_map.hh>
#include <corridor_planner/grid_graph.hh>

#include <map>
#include <stdexcept>

namespace corridor_planner {
    /** An implementation of the plain D* algorithm. DStar::costOfClass is the
     * method which transforms traversability value into a floating-point cost
     * value
     */
    class DStar
    {
    public:
        struct internal_error : public std::exception
        {
            char const* m_message;
            internal_error(char const* msg)
                : m_message(msg) {}
            ~internal_error() throw() { };

            char const* what() const throw() { return m_message; }
        };

        struct Cost {
            float value;

            Cost(float value) : value(value) {}

            bool operator < (Cost const& other) const  { return value - other.value < -0.0001; }
            bool operator <= (Cost const& other) const { return !(*this > other); }
            bool operator > (Cost const& other) const  { return value - other.value > 0.0001; }
            bool operator >= (Cost const& other) const { return !(*this < other); }
            bool operator == (Cost const& other) const { return std::fabs(value - other.value) <= (value / 10000); }
            bool operator != (Cost const& other) const { return !(*this == other); }
            Cost operator - (Cost const& other) const
            { return Cost(value - other.value); }
            Cost operator + (Cost const& other) const
            { return Cost(value + other.value); }
        };

    private:
        /** The underlying map we are acting on */
        TraversabilityMap& m_map;

        float m_cost_of_class[TraversabilityMap::CLASSES_COUNT];

        /** The GridGraph object we use to store the algorithm state. The float
         * value of a node in this graph stores the cost of the path from that
         * cell to the goal
         */
        GridGraph m_graph;

        /** The current goal */
        int m_goal_x, m_goal_y;

        typedef std::multimap<Cost, PointID, std::less<Cost>,
                pool_allocator< std::pair<Cost, PointID> > > OpenFromCost;
        OpenFromCost m_open_from_cost;
        typedef std::map<PointID, Cost, std::less<PointID>,
                pool_allocator< std::pair<PointID, Cost> > > OpenFromNode;
        OpenFromNode m_open_from_node;

    public:
        DStar(TraversabilityMap& map, TerrainClasses const& classes = TerrainClasses());

        /* Insert the following point in the open list, using the given value
         * as ordering value
         */
        Cost insert(int x, int y, Cost value);

        /** The graph object which is used to store D*'s results */
        GridGraph& graph();

        int getGoalX() const { return m_goal_x; }
        int getGoalY() const { return m_goal_y; }

        /** The graph object which is used to store D*'s results */
        GridGraph const& graph() const;

        /** Initializes the algorithm for the given position and goal */
        void initialize(int goal_x, int goal_y);

        /** Announce that the given cell has been updated in the traversability
         * map */
        float updated(int x, int y);

        /** Update the trajectories for the given position */
        void update();

        /** True if \c it points to a cell which has never been considered by
         * the algorithm */
        bool isNew(NeighbourConstIterator it) const;

        /** True if the pointed-to cell of \c it is currently in the open set */
        bool isOpened(NeighbourConstIterator it) const;

        /** True if \c x and \c y define a cell which has never been considered
         * by the algorithm */
        bool isNew(int x, int y) const;

        /** True if (\c x, \c y) is currently in the open set */
        bool isOpened(int x, int y) const;

        /** True if the pointed-to cell of \c it is neither new nor opened */
        bool isClosed(NeighbourConstIterator it) const
        { return isClosed(it.x(), it.y()); }

        /** True if (\c x, \c y) is neither new nor opened */
        bool isClosed(int x, int y) const { return !isNew(x, y) && !isOpened(x, y); }

        /** Returns the basic cost associated with the given terrain class */
        float costOfClass(int i) const;

        /** Computes the cost of crossing the edge represented by \c it
         *
         * The strategy is the following:
         * <ul>
         *  <li>use (a + b) / 2, where a and b are the costs of the source and
         *     target cells, if we are going straight
         *  <li>use (a + b + c + d) / 2, where a and b are the costs of the source
         *     and target cells and c and d are the costs of the two adjacent cells.
         *     This is used if we are going in diagonal.
         * </ul>
         */
        float costOf(NeighbourConstIterator it) const;

        /** Returns the cost of (x, y) as stored in the open list. If the node
         * is not in the open list, the boolean returned is false. Otherwise,
         * the boolean is true and the float value is the new cost of the node
         *
         * If \c check_consistency is true, then the method performs a
         * consistency check on the open list. It is quite costly and should
         * be used only for testing purposes
         */
        std::pair<float, bool> updatedCostOf(int x, int y, bool check_consistency = false) const;

        /** Sets the traversability class to \c klass for the given cell */
        void setTraversability(int x, int y, int klass);

        /** Checks that the current solution is consistent. It raises internal_error
         * if it is not the case */
        bool checkSolutionConsistency() const;
    };
}

#endif

