#ifndef NAV_GRID_GRAPH_HH
#define NAV_GRID_GRAPH_HH

#include <corridor_planner/grid_map.hh>
#include <corridor_planner/point.hh>

#include <boost/type_traits.hpp>
#include <boost/cstdint.hpp>

namespace corridor_planner
{
    class GridGraph;

    /** Base class for iteration around GridGraph cells. It is the common part
     * of NeighbourIterator and NeighbourConstIterator, the two classes you should
     * be using.
     *
     * @see GridGraph::parentsBegin GridGraph::parentsEnd GridGraph::neighboursBegin GridGraph::neighboursEnd
     */
    template<typename GraphClass>
    class NeighbourGenericIterator
    {
        friend class NeighbourGenericIterator<typename boost::add_const<GraphClass>::type>;

    protected:
        /** The underlying graph */
        GraphClass* m_graph;

        /** The position of the node whose neighbours we are visiting */
        int m_x, m_y;

        /** A bitfield which describes the remaining neighbours we should
         * visit. The iterator will not stop on neighbours for which a 0
         * is set in the mask.
         *
         * See the code in findNextNeighbour
         */
        int m_mask;

        /** the neighbour we are currently visiting. It is an integer between 1
         * and 9 (inclusive). The values between 1 and 8 describe the actual
         * relation between the central node in (m_x, m_y) and the currently
         * visited neighbour. 9 is the end of iteration.
         */
        int m_neighbour;

        static const int END_NEIGHBOUR = 9;
        static const int s_x_offsets[9];
        static const int s_y_offsets[9];

        /** Updates m_neighbour and m_mask to advance the iterator to the next
         * cell which should be visited, or to the end of iteration if no
         * cell remains
         */
        void findNextNeighbour();

    public:
        NeighbourGenericIterator()
            : m_graph(0), m_neighbour(END_NEIGHBOUR) {}
        NeighbourGenericIterator(GraphClass& graph, int x, int y, int mask);
        NeighbourGenericIterator(NeighbourGenericIterator<typename boost::remove_const<GraphClass>::type> const& other)
        {
            m_graph     = other.m_graph;
            m_x         = other.m_x;
            m_y         = other.m_y;
            m_mask      = other.m_mask;
            m_neighbour = other.m_neighbour;
        }

        int getMask() const { return m_mask; }

        /** The index of the neighbour we are currently visiting. 0 is
         * straight-right and it turns counter-clockwise (i.e. 3 is top-left).
         */
        int getNeighbourIndex() const { return m_neighbour - 1; }

        /** The neighbour we are currently visiting, as a value defined
         * in GridGraph::RELATIONS */
        int getNeighbour() const { return 1 << (m_neighbour - 1); }

        /** The position of the node on which we are iterating */
        float getSourceValue() const;
        PointID getSourcePoint() const { return PointID(sourceX(), sourceY()); }
        int sourceX() const { return m_x; }
        int sourceY() const { return m_y; }

        /** The position of the currently-iterated neighbour */
        PointID getTargetPoint() const { return PointID(x(), y()); }
        int x() const { return m_x + s_x_offsets[m_neighbour]; }
        int y() const { return m_y + s_y_offsets[m_neighbour]; }

        /** The value of the currently-iterated neighbour */
        float getValue() const;
        float getTargetValue() const { return getValue(); }

        /** True if the source of this iterator is the parent of the currently
         * pointed-to cell
         */
        bool sourceIsParent() const;

        /** Advance the iteration one step */
        NeighbourGenericIterator<GraphClass>& operator++()
        {
            ++m_neighbour;
            m_mask >>= 1;
            findNextNeighbour();
            return *this;
        };

        /** True if *this and \c other represent the same position in
         * iteration. It means that they both point to the same underlying cell
         * in the graph \b and are iterating around the same center node
         */
        bool operator == (NeighbourGenericIterator const& other) const
        {
            return ((isEnd() && other.isEnd())
                    || (m_neighbour == other.m_neighbour
                && m_graph == other.m_graph
                && m_x == other.m_x
                && m_y == other.m_y));
        }
        /** The inverse of ==
         */
        bool operator != (NeighbourGenericIterator const& other) const { return !(*this == other); }

        /** True if this iterator is a past-the-end iterator */
        bool isEnd() const { return m_neighbour >= END_NEIGHBOUR; }
    };

    template<typename GraphClass>
    const int NeighbourGenericIterator<GraphClass>::s_x_offsets[9] = { 0, 1, 1, 0, -1, -1, -1,  0,  1 };
    template<typename GraphClass>
    const int NeighbourGenericIterator<GraphClass>::s_y_offsets[9] = { 0, 0, 1, 1,  1,  0, -1, -1, -1 };

    class NeighbourIterator : public NeighbourGenericIterator<GridGraph>
    {
    public:
        NeighbourIterator()
            : NeighbourGenericIterator<GridGraph>() {}
        NeighbourIterator(GridGraph& graph, int x, int y, int mask)
            : NeighbourGenericIterator<GridGraph>(graph, x, y, mask) {}
        NeighbourIterator(NeighbourGenericIterator<GridGraph> const& source)
            : NeighbourGenericIterator<GridGraph>(source) {}

        /* Changes the value of the currently pointed-to cell */
        void setValue(float value);
        void setTargetValue(float value) { return setValue(value); };
        /* Make the source of this iteration the parent of the currently
         * pointed-to cell */
        void setSourceAsParent();
        /* Make pointed-to cell the parent of the source of this iteration
         */
        void setTargetAsParent();
    };

    class NeighbourConstIterator : public NeighbourGenericIterator<GridGraph const>
    {
    public:
        NeighbourConstIterator()
            : NeighbourGenericIterator<GridGraph const>() {}
        NeighbourConstIterator(GridGraph const& graph, int x, int y, int mask)
            : NeighbourGenericIterator<GridGraph const>(graph, x, y, mask) {}
        NeighbourConstIterator(NeighbourIterator const& source)
            : NeighbourGenericIterator<GridGraph const>(source) {}
    };

    /** Objects of this class represent a DAG based on regular grids. Each node
     * in the graph has 8 neighbours, and given a node, only itss parents are
     * available.
     *
     * Finally, one single floating-point value can be stored per node.
     */
    class GridGraph : public GridMap
    {
    public:
        typedef NeighbourIterator iterator;
        typedef NeighbourConstIterator const_iterator;

        /** Enum which describes the mapping between the bits in m_parents and
         * the actual neighbour direction.
         */
        enum RELATIONS {
            RIGHT        = 1,
            TOP_RIGHT    = 2,
            TOP          = 4,
            TOP_LEFT     = 8,
            LEFT         = 16,
            BOTTOM_LEFT  = 32,
            BOTTOM       = 64,
            BOTTOM_RIGHT = 128
        };

        static const int DIR_STRAIGHT = RIGHT | TOP | LEFT | BOTTOM;
        static const int DIR_DIAGONAL = TOP_RIGHT | TOP_LEFT | BOTTOM_LEFT | BOTTOM_RIGHT;
        static const int DIR_ALL = DIR_STRAIGHT | DIR_DIAGONAL;

    private:
        int m_width, m_height;

        /** Array which encodes the parents of each node. The array is stored
         * row-first (i.e. the value for (x, y) is at (y * m_xsize + x).
         *
         * The parents are encoded as a bitfield. The least significant bit is
         * the node on the same line than the considered node, and then we turn
         * counter-clockwise. See the RELATIONS enum.
         *
         */
        std::vector<boost::uint8_t> m_parents;


        /** Array which stores the cost value for each node.
         */
        std::vector<float> m_values;

    public:
        explicit GridGraph(size_t width = 0, size_t height = 0, float value = 0);

        int getXSize() const { return m_width; }
        int getWidth() const { return getXSize(); }
        int getYSize() const { return m_height; }
        int getHeight() const { return getYSize(); }

        /** Clears all edges in the graph, and initialize the floating-point
         * value to the given \c new_value */
        void clear(float new_value);

        /** Returns the floating point value stored for the (x, y) cell */
        float    getValue(size_t x, size_t y) const;
        /** Returns the floating point value stored for the (x, y) cell */
        void     setValue(size_t x, size_t y, float value);

        /** Returns the bit-mask describing what are the parents of the cell
         * at (x, y). See RELATIONS.
         */
        boost::uint8_t  getParents(size_t x, size_t y) const;
        /** Returns the bit-mask describing what are the parents of the cell
         * at (x, y). See RELATIONS.
         */
        boost::uint8_t& getParents(size_t x, size_t y);
        /** Adds a new parent for the cell at (x, y). See RELATIONS for the
         * set of possible values.
         */
        void     setParent(size_t x, size_t y, boost::uint8_t new_parent);
        /** Sets the set of parents to the given bitfield */
        void     setParents(size_t x, size_t y, boost::uint8_t mask);
        /** Removes a parent for the cell at (x, y). See RELATIONS for the set
         * of possible values.
         */
        void     clearParent(size_t x, size_t y, boost::uint8_t old_parent);

        /** Returns a NeighbourIterator object which allows to iterate on the
         * parents of (x, y). See also getParents.
         */
        iterator parentsBegin(size_t x, size_t y);
        /** Returns a NeighbourIterator object which allows to iterate on the
         * parents of (x, y). See also getParents.
         */
        const_iterator parentsBegin(size_t x, size_t y) const;
        /** Returns a NeighbourIterator object which allows to iterate on the
         * neighbours of (x, y). Neighbours are the cells directly adjacent to
         * the considered cell.
         */
        iterator neighboursBegin(size_t x, size_t y, int mask = DIR_ALL);
        /** Returns a NeighbourIterator object which allows to iterate on the
         * neighbours of (x, y). Neighbours are the cells directly adjacent to
         * the considered cell.
         */
        const_iterator neighboursBegin(size_t x, size_t y, int mask = DIR_ALL) const;

        /** Returns the iterator pointing to the given neighbour of (x, y) */
        NeighbourConstIterator getNeighbour(size_t x, size_t y, int neighbour) const;

        /** Save the graph in +io+ in the following format:
         * <pre>
         *  xSize ySize
         *  x y value parent_x parent_y
         *  ...
         * </pre>
         *
         * The parent_x and parent_y values are not present if the node
         * has no parent
         */
        void save(std::ostream& io) const;
    };

}
#endif

