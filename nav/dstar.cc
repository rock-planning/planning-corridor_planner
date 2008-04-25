#include <limits>
#include "dstar.hh"
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <cassert>
#include <algorithm>

#include <iostream>

using namespace Nav;
using std::make_pair;
using boost::tie;

namespace Nav {
    static const int OPPOSITE_RELATIONS[] = 
        { 0,
        GridGraph::LEFT,
        GridGraph::BOTTOM_LEFT,
        GridGraph::BOTTOM,
        GridGraph::BOTTOM_RIGHT,
        GridGraph::RIGHT,
        GridGraph::TOP_RIGHT,
        GridGraph::TOP,
        GridGraph::TOP_LEFT };
}

TraversabilityMap::TraversabilityMap(size_t width, size_t height, uint8_t init)
    : GridMap(width, height)
    , m_values((width * height + 1) / 2, init) { }

void TraversabilityMap::fill(uint8_t value)
{ 
    value &= 0xF;
    std::fill(m_values.begin(), m_values.end(), (value | value << 4));
}

uint8_t TraversabilityMap::getValue(size_t id) const
{
    int array_idx = id / 2;
    int shift = (id & 1) * 4;
    return (m_values[array_idx] & (0xF << shift)) >> shift;
}
uint8_t TraversabilityMap::getValue(size_t x, size_t y) const
{ return getValue(getCellID(x, y)); }
void TraversabilityMap::setValue(size_t id, uint8_t value)
{
    int array_idx = id / 2;
    int shift = (id & 1) * 4;
    m_values[array_idx] = (m_values[array_idx] & ~(0xF << shift)) | (value << shift);
}
void TraversabilityMap::setValue(size_t x, size_t y, uint8_t value)
{ return setValue(getCellID(x, y), value); }

template<typename GC>
NeighbourGenericIterator<GC>::NeighbourGenericIterator(GC& graph, int x, int y, int mask)
    : m_graph(&graph), m_x(x), m_y(y)
    , m_mask(mask)
    , m_neighbour(1)
{
    m_neighbour = 1;
    findNextNeighbour();
}
template<typename GC>
float NeighbourGenericIterator<GC>::getValue() const { return m_graph->getValue(x(), y()); }
template<typename GC>
bool NeighbourGenericIterator<GC>::sourceIsParent() const 
{ return m_graph->getParents(x(), y()) & Nav::OPPOSITE_RELATIONS[m_neighbour]; }

template<typename GC>
void NeighbourGenericIterator<GC>::findNextNeighbour()
{
    /* Initialize m_neighbour to the value of the first non-zero bit
     * in the parent field of (x, y)
     */
    while( ((m_mask & 0x1) == 0) && (m_neighbour < 9) )
    {
        m_mask >>= 1;
        m_neighbour++;
    }
}

template class NeighbourGenericIterator<Nav::GridGraph const>;
template class NeighbourGenericIterator<Nav::GridGraph>;

void NeighbourIterator::setValue(float value)
{ m_graph->setValue(x(), y(), value); }
void NeighbourIterator::setSourceAsParent()
{ 
    m_graph->setParents(x(), y(), Nav::OPPOSITE_RELATIONS[m_neighbour]);
    m_graph->clearParent(sourceX(), sourceY(), getNeighbour());
}
void NeighbourIterator::setTargetAsParent()
{ 
    m_graph->clearParent(x(), y(), Nav::OPPOSITE_RELATIONS[m_neighbour]);
    m_graph->setParents(sourceX(), sourceY(), getNeighbour());
}


GridGraph::GridGraph(size_t width, size_t height, float value)
    : GridMap(width, height)
    , m_parents(width * height, 0)
    , m_values(width * height, value)
{ }

void GridGraph::clear(float new_value)
{ 
    fill(m_parents.begin(), m_parents.end(), 0);
    fill(m_values.begin(), m_values.end(), new_value);
}

float  GridGraph::getValue(size_t x, size_t y) const
{ return m_values[y * m_xsize + x]; }
void GridGraph::setValue(size_t x, size_t y, float value)
{ m_values[y * m_xsize + x] = value; }

uint8_t  GridGraph::getParents(size_t x, size_t y) const
{ return m_parents[y * m_xsize + x]; }
uint8_t& GridGraph::getParents(size_t x, size_t y)
{ return m_parents[y * m_xsize + x]; }
void GridGraph::setParent(size_t x, size_t y, uint8_t new_parent)
{ getParents(x, y) |= new_parent; }
void GridGraph::setParents(size_t x, size_t y, uint8_t mask)
{ getParents(x, y) = mask; }
void GridGraph::clearParent(size_t x, size_t y, uint8_t old_parent)
{ getParents(x, y) &= ~old_parent; }

NeighbourIterator GridGraph::parentsBegin(size_t x, size_t y)
{ return NeighbourIterator(*this, x, y, getParents(x, y)); }
NeighbourConstIterator GridGraph::parentsBegin(size_t x, size_t y) const
{ return NeighbourConstIterator(*this, x, y, getParents(x, y)); }
NeighbourIterator GridGraph::parentsEnd() const
{ return NeighbourIterator(); }
static int neighboursMask(int x, int y, int xsize, int ysize)
{
    int mask = 0xFF;
    mask &= ~((x == 0) * (GridGraph::BOTTOM_LEFT | GridGraph::LEFT | GridGraph::TOP_LEFT));
    mask &= ~((x == xsize - 1) * (GridGraph::BOTTOM_RIGHT | GridGraph::RIGHT | GridGraph::TOP_RIGHT));
    mask &= ~((y == 0) * (GridGraph::BOTTOM_LEFT | GridGraph::BOTTOM | GridGraph::BOTTOM_RIGHT));
    mask &= ~((y == ysize - 1) * (GridGraph::TOP_LEFT | GridGraph::TOP | GridGraph::TOP_RIGHT));
    return mask;
}
NeighbourIterator GridGraph::neighboursBegin(size_t x, size_t y)
{ return NeighbourIterator(*this, x, y, neighboursMask(x, y, m_xsize, m_ysize)); }
NeighbourConstIterator GridGraph::neighboursBegin(size_t x, size_t y) const
{ return NeighbourConstIterator(*this, x, y, neighboursMask(x, y, m_xsize, m_ysize)); }
NeighbourIterator GridGraph::neighboursEnd() const
{ return NeighbourIterator(); }

NeighbourConstIterator GridGraph::getNeighbour(size_t x, size_t y, int neighbour) const
{ return NeighbourConstIterator(*this, x, y, neighbour); }


namespace Nav {
    static const float COST_GROWTH_1 = 0.3;
    static const float COST_GROWTH_0 = 0.01;
    static const float DIAG_FACTOR = sqrt(2);
}

DStar::DStar(TraversabilityMap const& map)
    : m_map(map)
    , m_graph(map.xSize(), map.ySize()) {}

GridGraph& DStar::graph()
{ return m_graph; }
GridGraph const& DStar::graph() const
{ return m_graph; }

void DStar::initialize(int goal_x, int goal_y, int pos_x, int pos_y)
{
    /** First, clear up everything */
    m_graph.clear(std::numeric_limits<float>::max());
    m_open_from_node.clear();
    m_open_from_cost.clear();
    m_goal_x = goal_x;
    m_goal_y = goal_y;

    m_graph.setValue(goal_x, goal_y, 0);
    updated(goal_x, goal_y);
    update(pos_x, pos_y);
}

float DStar::costOfClass(int klass)
{
    float p = static_cast<float>(klass) / (TraversabilityMap::CLASSES_COUNT - 1);
    float sigma = p * (COST_GROWTH_1 - COST_GROWTH_0) + COST_GROWTH_0;
    return std::pow(2, (1 - p) / sigma);
}
float DStar::costOf(size_t x, size_t y) const
{ return costOfClass(m_map.getValue(x, y)); }

float DStar::costOf(NeighbourConstIterator it) const
{
    float a = costOf(it.sourceX(), it.sourceY());
    float b = costOf(it.x(), it.y());

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

    float c = costOf(next.x(), next.y());
    float d = costOf(prev.x(), prev.y());
    return (a + b + c + d) / 2 * Nav::DIAG_FACTOR;
}

float DStar::updated(int x, int y)
{ 
    // Compute the new cost of the cell, based on its currently known parent
    // (or 0 if there is no parent yet), and add it to the open list.
    NeighbourConstIterator parent = m_graph.parentsBegin(x, y);
    float new_cost;
    if (parent == m_graph.parentsEnd())
        new_cost = costOf(x, y);
    else
        new_cost = m_graph.getValue(parent.x(), parent.y()) + costOf(parent);

    return insert(x, y, new_cost);
}

float DStar::insert(int x, int y, float cost)
{
    PointID point_id = { x, y };
    OpenFromNode::iterator it = m_open_from_node.find(point_id);
    if (it != m_open_from_node.end())
    {
        // The cell is already in the open list. Update it.
        float old_cost = it->second;
        if (cost >= old_cost) return old_cost;
        OpenFromCost::iterator it_cost, end;
        boost::tie(it_cost, end) = m_open_from_cost.equal_range(old_cost);
        while (it_cost->second != point_id && it_cost != end)
            ++it_cost;

        assert(it_cost != end);

        m_open_from_cost.erase(it_cost);
        it->second = cost;
    }
    else
        m_open_from_node.insert( make_pair(point_id, cost) );

    m_open_from_cost.insert( make_pair(cost, point_id) );
    return cost;
}

std::pair<float, bool> DStar::updatedCostOf(int x, int y, bool check_consistency) const
{
    PointID point_id = { x, y };
    OpenFromNode::const_iterator it_node = m_open_from_node.find(point_id);
    if (it_node == m_open_from_node.end())
        return make_pair(0, false);

    if (check_consistency)
    {
        if (m_open_from_node.size() != m_open_from_cost.size())
            throw internal_error();

        float cost = it_node->second;
        OpenFromCost::const_iterator it_cost, end_cost;
        bool already_seen;
        for (it_cost = m_open_from_cost.begin(); it_cost != m_open_from_cost.end(); ++it_cost)
        {
            if (it_cost->second == point_id)
            {
                if (already_seen || it_cost->first != cost)
                    throw internal_error();
                else
                    already_seen = true;
            }
        }
    }
    return make_pair(it_node->second, true);
}

void DStar::update(int pos_x, int pos_y)
{
    NeighbourIterator const end = m_graph.neighboursEnd();
    static const float tolerance = 0.001;
    while (!m_open_from_cost.empty())
    {
        /* Remove the top item of the open list */
        OpenFromCost::iterator it_cost = m_open_from_cost.begin();
        float new_cost   = it_cost->first;
        PointID point_id = it_cost->second;
        float old_cost   = m_graph.getValue(point_id.x, point_id.y);
        m_open_from_cost.erase(it_cost);
        m_open_from_node.erase(point_id);

        if ((new_cost - old_cost) < -tolerance)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); it != end; ++it)
            {
                float neighbour_cost = it.getValue() + costOf(it);
                if (it.getValue() < new_cost && old_cost > neighbour_cost)
                {
                    it.setTargetAsParent();
                    m_graph.setValue(it.sourceX(), it.sourceY(), neighbour_cost);
                    old_cost = neighbour_cost;
                }
            }
        }

        if (fabs(new_cost - old_cost) < tolerance)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); it != end; ++it)
            {
                float neighbour_cost = old_cost + costOf(it);
                if (it.getValue() == std::numeric_limits<float>::max() ||
                        it.sourceIsParent() && fabs(it.getValue() - neighbour_cost) < tolerance ||
                        !it.sourceIsParent() && it.getValue() > neighbour_cost)
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), neighbour_cost);
                }
            }
        }
        else
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); it != end; ++it)
            {
                float edge_cost = costOf(it);
                PointID target = {it.x(), it.y()};
                if (it.getValue() == std::numeric_limits<float>::max() ||
                        it.sourceIsParent() && fabs(it.getValue() - old_cost - edge_cost) < tolerance)
                {
                    it.setSourceAsParent();
                    insert(it.x(), it.y(), old_cost + edge_cost);
                }
                else if (!it.sourceIsParent() && it.getValue() > old_cost + edge_cost)
                {
                    insert(it.sourceX(), it.sourceY(), old_cost);
                }
                else if (!it.sourceIsParent() && 
                        old_cost > it.getValue() + edge_cost &&
                        !m_open_from_node.count(target) &&
                        it.getValue() > new_cost)
                {
                    insert(it.x(), it.y(), it.getValue());
                }
            }
        }
    }
}

