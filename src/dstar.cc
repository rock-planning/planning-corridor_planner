#include "dstar.hh"

#include <boost/tuple/tuple.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace nav;
// using namespace boost::lambda;
// using boost::tie;
// using boost::uint8_t;

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


namespace nav {
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
    return (a + b + c + d) / 2 * nav::DIAG_FACTOR;
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
    //if (m_map.getValue(x, y) == 0)
    //    return m_graph.getValue(x, y);

    Cost old_cost = m_graph.getValue(x, y);
    m_graph.setValue(x, y, new_cost.value);

    PointID point_id(x, y);

    OpenFromNode::iterator it = m_open_from_node.find(point_id);
    if (it != m_open_from_node.end())
    {
        // The cell is already in the open list. Update it.
        Cost old_cost = it->second;
        if (new_cost >= old_cost)
            return old_cost;

        OpenFromCost::iterator it_cost, end;
        boost::tie(it_cost, end) = m_open_from_cost.equal_range(old_cost);
        while (it_cost->second != point_id && it_cost != end)
            ++it_cost;

        assert(it_cost != end);

        m_open_from_cost.erase(it_cost);
        it->second = new_cost;
    }
    else
    {
        if (new_cost > old_cost)
            new_cost = old_cost;
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
    while (!m_open_from_cost.empty())
    {
        /* Remove the top item of the open list */
        OpenFromCost::iterator it_cost = m_open_from_cost.begin();
        PointID point_id  = it_cost->second;
        Cost new_cost     = it_cost->first;
        Cost old_cost     = m_graph.getValue(point_id.x, point_id.y);
        m_open_from_cost.erase(it_cost);
        m_open_from_node.erase(point_id);

        if (new_cost < old_cost)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); !it.isEnd(); ++it)
            {
                Cost neighbour_cost = it.getValue() + costOf(it);
                if (Cost(it.getValue()) < new_cost && old_cost > neighbour_cost)
                {
                    it.setTargetAsParent();
                    m_graph.setValue(it.sourceX(), it.sourceY(), neighbour_cost.value);
                    old_cost = neighbour_cost;
                }
            }
        }
        if (new_cost == old_cost)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); !it.isEnd(); ++it)
            {
                Cost neighbour_cost = old_cost + costOf(it);
                if (isNew(it) ||
                        (it.sourceIsParent()  && Cost(it.getValue()) != neighbour_cost) ||
                        (!it.sourceIsParent() && Cost(it.getValue()) > neighbour_cost))
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), neighbour_cost);
                }
            }
        }
        else
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); !it.isEnd(); ++it)
            {
                float edge_cost = costOf(it);
                PointID target(it.x(), it.y());
                if (isNew(it) ||
                        (it.sourceIsParent() && Cost(it.getValue()) != old_cost + edge_cost))
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), old_cost + edge_cost);
                }
                else if (!it.sourceIsParent())
                {
                    if (Cost(it.getValue()) > old_cost + edge_cost)
                        insert(it.sourceX(), it.sourceY(), old_cost);
                    else if (!it.sourceIsParent() && 
                            old_cost > it.getValue() + edge_cost &&
                            !m_open_from_node.count(target) &&
                            Cost(it.getValue()) > new_cost)
                    {
                        insert(it.x(), it.y(), it.getValue());
                    }
                }
            }
        }
    }
}

