#include "dstar.hh"

#include <boost/tuple/tuple.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace corridor_planner;

ostream& operator << (ostream& io, PointID const& p)
{ 
    io << "[" << p.x << ", " << p.y << "]";
    return io;
}
ostream& operator << (ostream& io, DStar::Cost const& v)
{ 
    io << v.value;
    return io;
}


namespace corridor_planner {
    static const float COST_GROWTH_1 = 0.3;
    static const float COST_GROWTH_0 = 0.01;
    static const float DIAG_FACTOR = sqrt(2);
}

DStar::DStar(TraversabilityMap& map, TerrainClasses const& classes)
    : m_map(map)
    , m_graph(map.xSize(), map.ySize(), std::numeric_limits<float>::max())
{
    if (classes.empty())
    {
        for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
            m_cost_of_class[i] = TraversabilityMap::CLASSES_COUNT + 1 - i;
    }
    else
    {
        float map_scale = map.getScale();

        for (int i = 0; i < TraversabilityMap::CLASSES_COUNT; ++i)
            m_cost_of_class[i] = 1000000;

        for (TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); ++it)
        {
            float speed = it->cost;
            if (speed == 0)
                m_cost_of_class[it->out] = 1000000;
            else
                m_cost_of_class[it->out] = map_scale / speed;

            cerr << " class " << it->out << " has cost " << m_cost_of_class[it->out] << endl;
        }
    }
}

GridGraph& DStar::graph()
{ return m_graph; }
GridGraph const& DStar::graph() const
{ return m_graph; }

void DStar::initialize(int goal_x, int goal_y)
{
    /** First, clear up everything */
    m_graph.clear(std::numeric_limits<float>::max());
    m_open_from_node.clear();
    m_open_from_cost.clear();
    m_goal_x = goal_x;
    m_goal_y = goal_y;

    m_graph.setValue(goal_x, goal_y, 0);
    insert(goal_x, goal_y, 0);
}

float DStar::costOfClass(int i) const { return m_cost_of_class[i]; }
float DStar::costOf(NeighbourConstIterator it) const
{
    float a = m_cost_of_class[m_map.getValue(it.sourceX(), it.sourceY())];
    float b = m_cost_of_class[m_map.getValue(it.x(), it.y())];

    if (it.getNeighbour() & GridGraph::DIR_STRAIGHT)
        return a + b;

    uint8_t next_neighbour = it.getNeighbour() << 1;
    uint8_t prev_neighbour = it.getNeighbour() >> 1;
    if (! next_neighbour)
        next_neighbour = GridGraph::RIGHT;
    else if (! prev_neighbour)
        prev_neighbour = GridGraph::BOTTOM_RIGHT;

    NeighbourConstIterator next = m_graph.getNeighbour(it.sourceX(), it.sourceY(), next_neighbour);
    NeighbourConstIterator prev = m_graph.getNeighbour(it.sourceX(), it.sourceY(), prev_neighbour);

    float c = m_cost_of_class[m_map.getValue(next.x(), next.y())];
    float d = m_cost_of_class[m_map.getValue(prev.x(), prev.y())];
    return (a + b + c + d) / 2 * corridor_planner::DIAG_FACTOR;
}

float DStar::updated(int x, int y)
{ 
    if (isClosed(x, y))
        insert(x, y, m_graph.getValue(x, y));

    for (NeighbourConstIterator it = m_graph.neighboursBegin(x, y); !it.isEnd(); ++it)
    {
        if (isClosed(it))
            insert(it.x(), it.y(), it.getValue());
    }
    return m_open_from_cost.begin()->first.value;
}

DStar::Cost DStar::insert(int x, int y, Cost new_cost)
{
    // The management of the open list is a bit more complicated than when
    // written in pseudo-code ...
    // 
    // The open list is made to sort the states by K-min, which is the minimum
    // value between H (the last value stored in the map) and *every* updates
    // ever processed since (x, y) has been added to the OPEN list
    //
    // What is means is that:
    //
    //  * if the node is not yet in the OPEN list, we just add it, and use the
    //    minimum between +new_cost+ and the cost stored in the map (a.k.a.
    //    H-cost)
    //  * if the node is already stored in the OPEN list, we have to update it
    //    if +new_cost+ is smaller than the already stored value
    Cost h_X = m_graph.getValue(x, y);
    // Update h(X)
    //
    // It is unclear if that is needed (from the paper), but seems to *be*
    // needed in practice ...
    m_graph.setValue(x, y, new_cost.value);

    PointID point_id(x, y);

    OpenFromNode::iterator it = m_open_from_node.find(point_id);
    if (it != m_open_from_node.end())
    {
        // The cell is already in the open list. Update it if needed
        Cost k_X = it->second;
        if (new_cost >= k_X)
        {
            // k_X is already the minimum, just return
            return k_X;
        }

        // We have to remove it first and then re-add it
        OpenFromCost::iterator it_cost, end;
        boost::tie(it_cost, end) = m_open_from_cost.equal_range(k_X);
        while (it_cost->second != point_id && it_cost != end)
            ++it_cost;
        assert(it_cost != end);

        m_open_from_cost.erase(it_cost);
        it->second = new_cost;
    }
    else
    {
        if (h_X < new_cost)
            new_cost = h_X;
        m_open_from_node.insert( make_pair(point_id, new_cost) );
    }

    m_open_from_cost.insert( make_pair(new_cost, point_id) );
    return new_cost;
}

bool DStar::checkSolutionConsistency() const
{
    bool has_error = false;
    for (int x = 0; x < (int)m_graph.xSize(); ++x)
        for (int y = 0; y < (int)m_graph.ySize(); ++y)
            if (x != getGoalX() || y != getGoalY())
            {
                NeighbourConstIterator parent = m_graph.parentsBegin(x, y);
                //if (!parent.isEnd() && m_map.getValue(x, y) == 0)
                //{
                //    has_error = true;
                //    fprintf(stderr, "node (%i, %i) is obstacle but has a parent\n", 
                //            x, y);
                //}
                if (m_graph.getValue(x, y) <= 0)
                {
                    has_error = true;
                    fprintf(stderr, "node (%i, %i) has negative cost: %e\n", 
                            x, y, m_graph.getValue(x, y));
                }
                if (Cost(m_graph.getValue(x, y)) != Cost(costOf(parent) + parent.getValue()))
                {
                    has_error = true;
                    fprintf(stderr, "saved cost and computed cost mismatch: %e != %e (parent == %e)\n", 
                            m_graph.getValue(x, y),
                            costOf(parent) + parent.getValue(),
                            parent.getValue());
                }
            }
            else
            {
                NeighbourConstIterator parent = m_graph.parentsBegin(x, y);
                if (!parent.isEnd())
                {
                    has_error = true;
                    fprintf(stderr, "the goal has a parent (%i, %i)\n",
                            parent.x(), parent.y());
                }
            }

    return !has_error;
}

std::pair<float, bool> DStar::updatedCostOf(int x, int y, bool check_consistency) const
{
    PointID point_id(x, y);
    OpenFromNode::const_iterator it_node = m_open_from_node.find(point_id);
    if (it_node == m_open_from_node.end())
        return make_pair(0, false);

    if (check_consistency)
    {
        if (m_open_from_node.size() != m_open_from_cost.size())
            throw internal_error("size of the two maps differ");

        Cost cost = it_node->second;
        OpenFromCost::const_iterator it_cost, end_cost;
        bool already_seen = false;
        for (it_cost = m_open_from_cost.begin(); it_cost != m_open_from_cost.end(); ++it_cost)
        {
            if (it_cost->second == point_id)
            {
                if (already_seen)
                    throw internal_error("duplicated node in open_from_cost");
                else if (it_cost->first != cost)
                    throw internal_error("mismatching cost between the two maps");
                else
                    already_seen = true;
            }
        }
    }
    return make_pair(it_node->second.value, true);
}


void DStar::setTraversability(int x, int y, int klass)
{
    m_map.setValue(x, y, klass);
    updated(x, y);
}

bool DStar::isNew(NeighbourConstIterator it) const 
{ return isNew(it.x(), it.y()); }
bool DStar::isNew(int x, int y) const 
{ return m_graph.getValue(x, y) == std::numeric_limits<float>::max(); }
bool DStar::isOpened(NeighbourConstIterator it) const
{ return isOpened(it.x(), it.y()); }
bool DStar::isOpened(int x, int y) const
{
    PointID id(x, y);
    return m_open_from_node.find(id) != m_open_from_node.end();
}

void DStar::update()
{
    /* This function is (in a loop) the implementation of the PROCESS-STATE()
     * function from the original D* algorithm
     *
     * We try to keep the notations as much as possible
     */
    while (!m_open_from_cost.empty())
    {
        /* Remove the top item of the open list
         *
         * Extracts the minimum state (i.e. point with minimum cost) and k_min
         * (minimum possible cost for this point)
         */
        OpenFromCost::iterator it_X = m_open_from_cost.begin();
        PointID X  = it_X->second;
        Cost k_old = it_X->first;

        /* This is the last stored cost (i.e. cost before we put min_state in
         * the OPEN list
         */
        Cost h_X   = m_graph.getValue(X.x, X.y);

        /* And remove both of them from the maps */
        m_open_from_cost.erase(it_X);
        m_open_from_node.erase(X);

        if (k_old < h_X)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(X.x, X.y); !it.isEnd(); ++it)
            {
                Cost h_Y = it.getValue();
                Cost neighbour_cost = h_Y + costOf(it);
                if (h_Y <= k_old && h_X > neighbour_cost)
                {
                    it.setTargetAsParent();
                    m_graph.setValue(it.sourceX(), it.sourceY(), h_X.value);
                    h_X = neighbour_cost.value;
                }
            }
        }
        if (k_old == h_X)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(X.x, X.y); !it.isEnd(); ++it)
            {
                Cost h_Y = it.getValue();
                Cost neighbour_cost = h_X + costOf(it);
                if (isNew(it) ||
                        (it.sourceIsParent()  && h_Y != neighbour_cost) ||
                        (!it.sourceIsParent() && h_Y > neighbour_cost))
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), neighbour_cost);
                }
            }
        }
        else
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(X.x, X.y); !it.isEnd(); ++it)
            {
                Cost h_Y = it.getValue();
                float edge_cost = costOf(it);
                PointID target(it.x(), it.y());
                if (isNew(it) ||
                        (it.sourceIsParent() && h_Y != h_X + edge_cost))
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), h_X + edge_cost);
                }
                else if (!it.sourceIsParent())
                {
                    if (h_Y > h_X + edge_cost)
                        insert(it.sourceX(), it.sourceY(), h_X);
                    else if (!it.sourceIsParent() && 
                            h_X > h_Y + edge_cost &&
                            isClosed(target.x, target.y) &&
                            h_Y > k_old)
                    {
                        insert(it.x(), it.y(), h_Y);
                    }
                }
            }
        }
    }
}

