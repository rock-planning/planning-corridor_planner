#include "traversability_map.hh"

#include <memory>
#include <gdal.h>
#include <gdal_priv.h>
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;
using namespace corridor_planner;

PointID TraversabilityMap::toLocal(Eigen::Vector3d const& v) const
{
    Eigen::Transform3d world_to_local(getLocalToWorld().inverse());
    Eigen::Vector3i raster3d = (world_to_local * v).cast<int>();
    return PointID(raster3d.x(), raster3d.y());
}

Eigen::Vector3d TraversabilityMap::toWorld(PointID const& v) const
{
    return getLocalToWorld() * v.toEigen();
}

TraversabilityMap* TraversabilityMap::load(std::string const& path, TerrainClasses const& classes)
{
    GDALAllRegister();

    auto_ptr<GDALDataset> set((GDALDataset*) GDALOpen(path.c_str(), GA_ReadOnly));
    if (!set.get())
        throw std::runtime_error("GDAL cannot open the specified file");

    GDALRasterBand* band = set->GetRasterBand(1);
    if (!band)
        throw std::runtime_error("there is no raster band in this file");

    double transform[6];
    set->GetGeoTransform(transform);

    Eigen::Transform3d local_to_world;
    local_to_world.matrix() <<
        transform[1], transform[2], 0, transform[0],
        transform[4], transform[5], 0, transform[3],
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix3d rotation, scaling;
    local_to_world.computeScalingRotation(&scaling, &rotation);
    if (fabs(fabs(scaling(0, 0) - fabs(scaling(1, 1)))) > 0.001)
        throw std::runtime_error("cannot use maps that have non-square pixels");

    int width  = band->GetXSize();
    int height = band->GetYSize();

    vector<uint8_t> data(width * height);
    band->RasterIO(GF_Read, 0, 0, width, height,
	    &data[0], width, height, GDT_Byte, 0, 0);

    if (!classes.empty())
    {
        int klass_map[256];
        for (int i = 0; i < 256; ++i)
            klass_map[i] = -1;
        for (TerrainClasses::const_iterator it = classes.begin(); it != classes.end(); ++it)
            klass_map[it->in] = it->out;

        for (int i = 0; i < width * height; ++i)
        {
            int out = klass_map[data[i]];
            if (out == -1)
            {
                cerr << "unknown class found: " << static_cast<int>(data[i]) << endl;
                return NULL;
            }
            data[i] = out;
        }
    }

    auto_ptr<TraversabilityMap> map(new TraversabilityMap(width, height,
                local_to_world, 0));
    map->fill(data);
    return map.release();
}

TraversabilityMap::TraversabilityMap(size_t width, size_t height, uint8_t init)
    : GridMap(width, height)
    , m_values((width * height + 1) / 2, init) { }

TraversabilityMap::TraversabilityMap(size_t width, size_t height,
        Eigen::Transform3d const& local_to_world, uint8_t init)
    : GridMap(width, height)
    , m_local_to_world(local_to_world)
    , m_values((width * height + 1) / 2, init)
{
    Eigen::Matrix3d rotation, scaling;
    local_to_world.computeScalingRotation(&scaling, &rotation);
    m_scale = scaling(0, 0);
}

float TraversabilityMap::getScale() const { return fabs(m_scale); }

void TraversabilityMap::fill(vector<uint8_t> const& values)
{
    m_values.resize(values.size() / 2);
    for (int i = 0; i < (int)m_values.size(); ++i)
    {
        m_values[i] = 
            static_cast<int>(values[2 * i]) |
            static_cast<int>(values[2 * i + 1] << 4);
    }
    if (values.size() % 2 == 1)
        m_values.push_back(values[values.size() - 1]);
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

