#ifndef NAV_GRID_MAP_HH
#define NAV_GRID_MAP_HH

namespace corridor_planner
{
    /** Basic tools for maps which are regular grids */
    class GridMap
    {
    protected:
        long m_xsize, m_ysize;

    public:
        explicit GridMap(long xsize = 0, long ysize = 0)
            : m_xsize(xsize), m_ysize(ysize) { }

        /** True if one of the dimensions is of zero size */
        bool empty() const { return m_xsize == 0 || m_ysize == 0; }

        /** The size, in cells, in the X direction */
        long xSize() const { return m_xsize; }
        /** The size, in cells, in the Y direction */
        long ySize() const { return m_ysize; }
        /** A numeric ID for the cell at (x, y) */
        long getCellID(long x, long y) const { return y * m_xsize + x; }
    };
}

#endif

