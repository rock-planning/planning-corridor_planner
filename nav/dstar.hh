#ifndef NAV_DSTAR_HPP
#define NAV_DSTAR_HPP

#include <vector>

namespace Nav {
    /** Objects of this class represent traversability maps. Traversability is
     * an integer value which can be represented on 4 bits (i.e. 16
     * traversability classes).
     */
    class TraversabilityMap
    {
        size_t m_width, m_height;
        std::vector<uint32_t> m_values;

    public:
        static const int CLASSES_COUNT = 16;

        TraversabilityMap(size_t width, size_t height);

        uint8_t getValue(size_t x, size_t y) const;
        void setValue(size_t x, size_t y, uint8_t value);
    };

    class GridGraph;
    class NeighbourIterator
    {
        /** The underlying graph */
        GridGraph* m_graph;

        /** The position of the node whose neighbours we are visiting */
        int m_x, m_y;
        /** A bitfield which describes the remaining neighbours we should
         * visit. If a 1 is found, the iterator stops on that neighbour, but it
         * ignores the ones for which 0 is set */
        int m_mask;

        /** the neighbour we are currently visiting. It is an integer between 1
         * and 9 (inclusive). The values between 1 and 8 describe the actual
         * relation between the central node in (m_x, m_y) and the currently
         * visited neighbour. 9 is the end of iteration.
         */
        int m_neighbour;

        static const int END_NEIGHBOUR = 9;
        static const int m_x_offsets[9];
        static const int m_y_offsets[9];

        void findNextNeighbour();

    public:
        NeighbourIterator()
            : m_graph(0), m_neighbour(END_NEIGHBOUR) {}
        NeighbourIterator(GridGraph& graph, int x, int y, int mask);

        int getMask() const { return m_mask; }

        /** The index of the neighbour we are currently visiting. Useful mostly
         * for debugging purposes */
        int getNeighbour() const { return m_neighbour; }

        /** The position of the node on which we are iterating */
        int nodeX() const { return m_x; }
        int nodeY() const { return m_y; }

        /** The position of the currently-iterated neighbour */
        int x() const { return m_x + m_x_offsets[m_neighbour]; }
        int y() const { return m_y + m_y_offsets[m_neighbour]; }

        /** The value of the currently-iterated neighbour */
        float& value();

        NeighbourIterator& operator++()
        {
            ++m_neighbour;
            m_mask >>= 1;
            findNextNeighbour();
            return *this;
        };

        bool operator == (NeighbourIterator const& other) const
        {
            return (m_neighbour == other.m_neighbour
                    || (m_neighbour >= END_NEIGHBOUR && other.m_neighbour >= END_NEIGHBOUR)
                && m_graph == other.m_graph
                && m_x == other.m_x
                && m_y == other.m_y);
        }
        bool operator != (NeighbourIterator const& other) const { return !(*this == other); }
    };


    /** Objects of this class represent a DAG based on regular grids. Each node
     * in the graph has 8 neighbours, and given a node, only itss parents are
     * available.
     *
     * Finally, one single floating-point value can be stored per node.
     */
    class GridGraph
    {
    public:
        typedef NeighbourIterator iterator;

        /** Enum which describes the mapping between the bits in m_parents and
         * the actual neighbour direction
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

    private:
        size_t m_width, m_height;
        /** Array which encodes the parents of each node. The array is stored
         * row-first (i.e. the value for (x, y) is at (y * m_width + x).
         *
         * The parents are encoded as a bitfield. The least significant bit is
         * the node on the same line than the considered node, and then we turn
         * counter-clockwise. See the RELATIONS enum.
         *
         */
        std::vector<uint8_t> m_parents;


        /** Array which encodes the parents of each node. The array is stored
         * row-first (i.e. the value for (x, y) is at (y * m_width + x).
         */
        std::vector<float> m_values;

    public:
        GridGraph(size_t width, size_t height);

        float getValue(size_t x, size_t y) const;
        float& getValue(size_t x, size_t y);

        uint8_t getParents(size_t x, size_t y) const;
        uint8_t& getParents(size_t x, size_t y);
        void setParent(size_t x, size_t y, uint8_t new_parent);
        void clearParent(size_t x, size_t y, uint8_t old_parent);

        iterator parentsBegin(size_t x, size_t y);
        iterator parentsEnd();
    };

    class DStar
    {
    };
}

#endif

