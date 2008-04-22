#include <limits>
#include "dstar.hh"
#include <cmath>

using namespace Nav;

TraversabilityMap::TraversabilityMap(size_t width, size_t height)
    : GridMap(width, height)
    , m_values((width * height + 1) / 2, 0)
{ }

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
float& NeighbourIterator::value() { return m_graph->getValue(x(), y()); }

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
float& GridGraph::getValue(size_t x, size_t y)
{ return m_values[y * m_xsize + x]; }

uint8_t  GridGraph::getParents(size_t x, size_t y) const
{ return m_parents[y * m_xsize + x]; }
uint8_t& GridGraph::getParents(size_t x, size_t y)
{ return m_parents[y * m_xsize + x]; }
void GridGraph::setParent(size_t x, size_t y, uint8_t new_parent)
{ getParents(x, y) |= new_parent; }
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
    static const float DIAG_FACTOR = sqrt(2) / 2;
}

DStar::DStar(TraversabilityMap const& map)
    : m_map(map)
    , m_graph(map.xSize(), map.ySize()) {}

GridGraph const& DStar::graph() const
{ return m_graph; }

void DStar::initialize(int goal_x, int goal_y, int pos_x, int pos_y)
{
    /** First, clear up everything */
    m_graph.clear(std::numeric_limits<float>::max());
    m_open_list.clear();
    m_goal_x = goal_x;
    m_goal_y = goal_y;

    m_open_list.insert( std::make_pair(0, m_graph.getCellID(goal_x, goal_y)) );
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

void DStar::updated(int x, int y)
{ 
}

void DStar::update(int pos_x, int pos_y)
{
    throw;
}

