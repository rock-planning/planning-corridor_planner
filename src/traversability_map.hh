#ifndef NAV_TRAVERSABILITY_MAP_HH
#define NAV_TRAVERSABILITY_MAP_HH

#include <corridor_planner/grid_map.hh>
#include <corridor_planner/terrain_classes.hh>
#include <corridor_planner/point.hh>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <boost/cstdint.hpp>

namespace corridor_planner
{
    /** Objects of this class represent traversability maps. Traversability is
     * an integer value which can be represented on 4 bits (i.e. 16
     * traversability classes).
     */
    class TraversabilityMap : public GridMap
    {
        Eigen::Transform3d m_local_to_world;

        float m_scale;

        /** The vector of values. Traversability is encoded on 4-bits fields, which
         * means that one uint8_t stores two cells. Moreover, values are stored
         * X-first, which means that the value for the cell (x, y) is at (y *
         * xsize + x).
         */
        std::vector<boost::uint8_t> m_values;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** How much classes can be stored in this map */
        static const int CLASSES_COUNT = 16;

        /** Creates a new map with the given size in the X and Y axis */
        TraversabilityMap(size_t xsize, size_t ysize, boost::uint8_t fill = 0);
        TraversabilityMap(size_t xsize, size_t ysize,
                Eigen::Transform3d const& local_to_world,
                boost::uint8_t fill = 0);

        /** Returns the size, in meters, of one pixel */
        float getScale() const;

        /** Transforms 3D coordinates in the world frame into 2D coordinates in
         * the raster frame
         */
        PointID toLocal(Eigen::Vector3d const& v) const;

        /** Transforms 2D coordinates in the raster frame into 3D coordinates in
         * the worldframe
         */
        Eigen::Vector3d toWorld(PointID const& v) const;

        /** Returns the transformation from raster to world frame */
        Eigen::Transform3d getLocalToWorld() const { return m_local_to_world; }

        /** Fills the map with the given traversability */
        void fill(boost::uint8_t value);
        void fill(std::vector<uint8_t> const& value);

        /** Returns the traversability value for the cell with the given ID */
        boost::uint8_t getValue(size_t id) const;
        /** Changes the traversability value for the cell with the given ID */
        void setValue(size_t id, boost::uint8_t value);
        /** Returns the traversability value for the cell at (x, y) */
        boost::uint8_t getValue(size_t x, size_t y) const;
        /** Changes the traversability value for the cell at (x, y) */
        void setValue(size_t x, size_t y, boost::uint8_t value);
        /** Creates a new map whose data is loaded from a GDAL-compatible file.
         * The values in the file are supposed to be floating-point values
         * in-between 0 and 1
         *
         * You MUST have called GDALAllRegister() yourself (for instance in
         * main()) before using that method. For instance, do
         *
         * <code>
         * #include <gdal.h>
         *
         * int main()
         * {
         *    GDALAllRegister();
         *    <blablabla do whatever you want, including using load()
         *    return 0;
         * }
         * </code>
         */
        static TraversabilityMap* load(std::string const& path, TerrainClasses const& classes);
    };
}

#endif
