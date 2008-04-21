#include "dstar.hh"

using namespace Nav;

TraversabilityMap::TraversabilityMap(size_t width, size_t height)
    : m_width(width), m_height(height)
    , m_values((width * height + 1) / 2, 0)
{ }

uint8_t TraversabilityMap::getValue(size_t x, size_t y) const
{
    int base_idx = y * m_width + x;
    int array_idx = base_idx / 2;
    int shift = (base_idx & 1) * 4;
    return (m_values[array_idx] & (0xF << shift)) >> shift;
}
void TraversabilityMap::setValue(size_t x, size_t y, uint8_t value)
{
    int base_idx = y * m_width + x;
    int array_idx = base_idx / 2;
    int shift = (base_idx & 1) * 4;
    m_values[array_idx] = (m_values[array_idx] & ~(0xF << shift)) | (value << shift);
}

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

GridGraph::GridGraph(size_t width, size_t height)
    : m_width(width), m_height(height)
    , m_parents(width * height, 0)
    , m_values(width * height, 0)
{ }

float  GridGraph::getValue(size_t x, size_t y) const
{ return m_values[y * m_width + x]; }
float& GridGraph::getValue(size_t x, size_t y)
{ return m_values[y * m_width + x]; }

uint8_t  GridGraph::getParents(size_t x, size_t y) const
{ return m_parents[y * m_width + x]; }
uint8_t& GridGraph::getParents(size_t x, size_t y)
{ return m_parents[y * m_width + x]; }
void GridGraph::setParent(size_t x, size_t y, uint8_t new_parent)
{ getParents(x, y) |= new_parent; }
void GridGraph::clearParent(size_t x, size_t y, uint8_t old_parent)
{ getParents(x, y) &= ~old_parent; }

NeighbourIterator GridGraph::parentsBegin(size_t x, size_t y)
{ return NeighbourIterator(*this, x, y, getParents(x, y)); }
NeighbourIterator GridGraph::parentsEnd()
{ return NeighbourIterator(); }

