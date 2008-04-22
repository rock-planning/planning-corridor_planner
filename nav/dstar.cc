#include "dstar.hh"

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

const int NeighbourIterator::m_x_offsets[9] = { 0, 1, 1, 0, -1, -1, -1,  0,  1 };
const int NeighbourIterator::m_y_offsets[9] = { 0, 0, 1, 1,  1,  0, -1, -1, -1 };

NeighbourIterator::NeighbourIterator(GridGraph& graph, int x, int y, int mask)
    : m_graph(&graph), m_x(x), m_y(y)
    , m_mask(mask)
    , m_neighbour(1)
{
    m_neighbour = 1;
    findNextNeighbour();
}
float& NeighbourIterator::value() { return m_graph->getValue(x(), y()); }
void NeighbourIterator::findNextNeighbour()
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
NeighbourIterator GridGraph::parentsEnd()
{ return NeighbourIterator(); }
NeighbourIterator GridGraph::neighboursBegin(size_t x, size_t y)
{ 
    int mask = 0xFF;
    mask &= ~((x == 0) * (BOTTOM_LEFT | LEFT | TOP_LEFT));
    mask &= ~((x == m_xsize - 1) * (BOTTOM_RIGHT | RIGHT | TOP_RIGHT));
    mask &= ~((y == 0) * (BOTTOM_LEFT | BOTTOM | BOTTOM_RIGHT));
    mask &= ~((y == m_ysize - 1) * (TOP_LEFT | TOP | TOP_RIGHT));
    return NeighbourIterator(*this, x, y, mask); 
}
NeighbourIterator GridGraph::neighboursEnd()
{ return NeighbourIterator(); }

