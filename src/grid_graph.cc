#include "grid_graph.hh"

using namespace corridor_planner;

namespace corridor_planner {
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
float NeighbourGenericIterator<GC>::getSourceValue() const { return m_graph->getValue(m_x, m_y); }
template<typename GC>
float NeighbourGenericIterator<GC>::getValue() const { return m_graph->getValue(x(), y()); }
template<typename GC>
bool NeighbourGenericIterator<GC>::sourceIsParent() const 
{ return m_graph->getParents(x(), y()) & corridor_planner::OPPOSITE_RELATIONS[m_neighbour]; }

template<typename GC>
void NeighbourGenericIterator<GC>::findNextNeighbour()
{
    /* Initialize m_neighbour to the value of the first non-zero bit
     * in the parent field of (x, y)
     */
    while ( ((m_mask & 0x1) == 0) && (m_neighbour < 9) )
    {
        m_mask >>= 1;
        m_neighbour++;
    }
}

template class NeighbourGenericIterator<corridor_planner::GridGraph const>;
template class NeighbourGenericIterator<corridor_planner::GridGraph>;

void NeighbourIterator::setValue(float value)
{ m_graph->setValue(x(), y(), value); }
void NeighbourIterator::setSourceAsParent()
{ 
    m_graph->setParents(x(), y(), corridor_planner::OPPOSITE_RELATIONS[m_neighbour]);
    m_graph->clearParent(sourceX(), sourceY(), getNeighbour());
}
void NeighbourIterator::setTargetAsParent()
{ 
    m_graph->clearParent(x(), y(), corridor_planner::OPPOSITE_RELATIONS[m_neighbour]);
    m_graph->setParents(sourceX(), sourceY(), getNeighbour());
}

GridGraph::GridGraph(size_t width, size_t height, float value)
    : GridMap(width, height)
    , m_width(width), m_height(height)
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
void GridGraph::save(std::ostream& io) const
{
    io << xSize() << " " << ySize() << std::endl;
    for (int y = 0; y < (int)ySize(); ++y)
    {
        for (int x = 0; x < (int)xSize(); ++x)
        {
            NeighbourConstIterator parent = parentsBegin(x, y);
            io << x << " " << y << " " << getValue(x, y);
            if (!parent.isEnd())
                io << " " << parent.x() << " " << parent.y();
            io << "\n";
        }
    }
}




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
NeighbourIterator GridGraph::neighboursBegin(size_t x, size_t y, int mask)
{ return NeighbourIterator(*this, x, y, mask & neighboursMask(x, y, m_xsize, m_ysize)); }
NeighbourConstIterator GridGraph::neighboursBegin(size_t x, size_t y, int mask) const
{ return NeighbourConstIterator(*this, x, y, mask & neighboursMask(x, y, m_xsize, m_ysize)); }
NeighbourConstIterator GridGraph::getNeighbour(size_t x, size_t y, int neighbour) const
{ return NeighbourConstIterator(*this, x, y, neighbour); }



