#include <limits>
#include "dstar.hh"
#include <cmath>
#include <cassert>
#include <algorithm>
#include <memory>
#include <boost/tuple/tuple.hpp>
#include <boost/lambda/lambda.hpp>
#include <gdal.h>
#include <gdal_priv.h>

#include <iostream>

using namespace std;
using namespace Nav;
using namespace boost::lambda;
using boost::tie;
using boost::uint8_t;

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

TraversabilityMap* TraversabilityMap::load(std::string const& path)
{
    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(path.c_str(), GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);

    int width  = band->GetXSize();
    int height = band->GetYSize();

    vector<float> data(width * height);
    band->RasterIO(GF_Read, 0, 0, width, height,
	    &data[0], width, height, GDT_Float32, 0, 0);

    auto_ptr<TraversabilityMap> map(new TraversabilityMap(width, height, 0));
    map->fill(data);
    return map.release();
}

TraversabilityMap::TraversabilityMap(size_t width, size_t height, uint8_t init)
    : GridMap(width, height)
    , m_values((width * height + 1) / 2, init) { }

void TraversabilityMap::fill(vector<float> const& values)
{
    for (int i = 0; i < (int)m_values.size(); ++i)
    {
        m_values[i] = 
            static_cast<int>(values[i] * (CLASSES_COUNT - 1)) |
            (static_cast<int>(values[2 * i + 1] * (CLASSES_COUNT - 1)) << 4);
    }
}
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
NeighbourConstIterator GridGraph::getNeighbour(size_t x, size_t y, int neighbour) const
{ return NeighbourConstIterator(*this, x, y, neighbour); }


namespace Nav {
    static const float COST_GROWTH_1 = 0.3;
    static const float COST_GROWTH_0 = 0.01;
    static const float DIAG_FACTOR = sqrt(2);
}

DStar::DStar(TraversabilityMap& map)
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
    insert(goal_x, goal_y, 0);
    update(pos_x, pos_y);
}

float DStar::costOfClass(int klass)
{
    return TraversabilityMap::CLASSES_COUNT + 1 - klass;
    // float p = static_cast<float>(klass) / (TraversabilityMap::CLASSES_COUNT - 1);
    // float sigma = p * (COST_GROWTH_1 - COST_GROWTH_0) + COST_GROWTH_0;
    // return std::pow(2, (1 - p) / sigma);
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
    
    for (NeighbourConstIterator it = m_graph.neighboursBegin(x, y); !it.isEnd(); ++it)
    {
        if (!isOpened(it) && !isNew(it))
            insert(it.x(), it.y(), it.getValue());
    }
    return m_open_from_cost.begin()->first.value;
}

DStar::Cost DStar::insert(int x, int y, Cost cost)
{
    PointID point_id(x, y);
    OpenFromNode::iterator it = m_open_from_node.find(point_id);
    if (it != m_open_from_node.end())
    {
        // The cell is already in the open list. Update it.
        Cost old_cost = it->second;
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

void DStar::checkSolutionConsistency()
{
    for (int x = 0; x < (int)m_graph.xSize(); ++x)
        for (int y = 0; y < (int)m_graph.ySize(); ++y)
            if (x != getGoalX() || y != getGoalY())
            {
                NeighbourConstIterator parent = m_graph.parentsBegin(x, y);
                if (parent.isEnd())
                    throw internal_error("a node has no parent");
                if (m_graph.getValue(x, y) <= 0)
                    throw internal_error("cost not strictly positive");
                if (Cost(m_graph.getValue(x, y)) != Cost(costOf(parent) + parent.getValue()))
                    throw internal_error("saved cost and computed cost mismatch");
            }
            else
            {
                NeighbourConstIterator parent = m_graph.parentsBegin(x, y);
                if (!parent.isEnd())
                    throw internal_error("goal has a parent");
            }

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

bool DStar::isNew(NeighbourConstIterator it) const { return it.getValue() == std::numeric_limits<float>::max(); }
bool DStar::isOpened(NeighbourConstIterator it) const {
    PointID id(it.x(), it.y());
    return m_open_from_node.find(id) != m_open_from_node.end();
}

void DStar::setSourceAsParent(NeighbourIterator it)
{
    it.setSourceAsParent();
    it.setValue(m_graph.getValue(it.sourceX(), it.sourceY()) + costOf(it));
}

void DStar::setTargetAsParent(NeighbourIterator it)
{
    it.setTargetAsParent();
    m_graph.setValue(it.sourceX(), it.sourceY(), it.getValue() + costOf(it));
}


void DStar::update(int pos_x, int pos_y)
{
    while (!m_open_from_cost.empty())
    {
        /* Remove the top item of the open list */
        OpenFromCost::iterator it_cost = m_open_from_cost.begin();
        Cost new_cost   = it_cost->first;
        PointID point_id = it_cost->second;
        Cost old_cost   = m_graph.getValue(point_id.x, point_id.y);
        m_open_from_cost.erase(it_cost);
        m_open_from_node.erase(point_id);

        if (new_cost < old_cost)
        {
            for (NeighbourIterator it = m_graph.neighboursBegin(point_id.x, point_id.y); !it.isEnd(); ++it)
            {
                Cost neighbour_cost = it.getValue() + costOf(it);
                if (Cost(it.getValue()) < new_cost && old_cost > neighbour_cost)
                {
                    setTargetAsParent(it);
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
                    setSourceAsParent(it);
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
                        (it.sourceIsParent() && Cost(it.getValue()) < old_cost + edge_cost))
                {
                    setSourceAsParent(it);
                    insert(it.x(), it.y(), old_cost + edge_cost);
                }
                else if (!it.sourceIsParent() && Cost(it.getValue()) > old_cost + edge_cost)
                {
                    insert(it.sourceX(), it.sourceY(), old_cost);
                }
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

std::set< std::pair<int, int> > DStar::solutionBorder(int x, int y, float expand) const
{
    typedef multimap<float, PointID> Border;
    typedef set<PointID> Inside;
    Border border;
    Inside inside;

    float min_limit = m_graph.getValue(x, y), max_limit = expand * min_limit;
    PointID start_id(x, y);
    border.insert(make_pair(max_limit, start_id));

    Border new_border;
    while(true)
    {
        // expand +solution+ until all elements in it have their cost greater
        // than cost*expand
        Border::iterator it = border.begin(), 
            end = border.upper_bound(max_limit);
        new_border.clear();

        // Nothing to expand further, go to the next element of the reference
        // path and start again
        if (it == end)
        {
            NeighbourConstIterator next_element = m_graph.parentsBegin(x, y);
            if (next_element.isEnd())
            {
                // Cannot go further than the goal, so stop here
                break;
            }
            max_limit = min_limit;
            x = next_element.x();
            y = next_element.y();
            min_limit = m_graph.getValue(x, y);
            border.insert(make_pair(min_limit, PointID(x, y)));
        }
        else
        {
            for (; it != end; ++it)
            {
                PointID p = it->second;
                inside.insert(p);
                for (NeighbourConstIterator n = m_graph.neighboursBegin(p.x, p.y); !n.isEnd(); ++n)
                {
                    PointID n_id = PointID(n.x(), n.y());
                    float   n_v  = n.getValue();
                    if (!inside.count(n_id) && n_v >= min_limit)
                    {
                        inside.insert(n_id);
                        new_border.insert(make_pair(n.getValue(), n_id));
                    }
                }
            }
            border.erase(border.begin(), end);
            border.insert(new_border.begin(), new_border.end());
        }
    }

    Inside border_set;
    for (Border::const_iterator it = border.begin(); it != border.end(); ++it)
    {
        PointID p(it->second);
        border_set.insert(p);
    }

    Inside shrink_set;
    do
    {
        for (Inside::const_iterator erase_it = shrink_set.begin();
                erase_it != shrink_set.end();
                ++erase_it)
        {
            inside.erase(*erase_it);
        }
        shrink_set.clear();

        for (Inside::iterator it = inside.begin(); it != inside.end(); ++it)
        {
            NeighbourConstIterator p_it = m_graph.parentsBegin(it->x, it->y);
            if (!p_it.isEnd())
            {
                PointID p_id = PointID(p_it.x(), p_it.y());
                if (!inside.count(p_id) || shrink_set.count(p_id))
                    shrink_set.insert(PointID(it->x, it->y));
            }
        }
    }
    while (!shrink_set.empty());

    set< pair<int, int> > ret;
    for (Inside::const_iterator it = inside.begin(); it != inside.end(); ++it)
    {
        PointID p(*it);
        if (!border_set.count(p))
            ret.insert(make_pair(p.x, p.y));
    }
    //for (Border::const_iterator it = border.begin(); it != border.end(); ++it)
    //{
    //    PointID p(it->second);
    //    ret.insert(make_pair(p.x, p.y));
    //}
    return ret;
}

