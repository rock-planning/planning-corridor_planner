#include "CorridorPlanVisualization.hpp"
#include "corridor_planner/corridors.hh"

#include <osg/Geometry>
#include <osg/Geode>

#include <envire/maps/Grids.hpp>

using namespace vizkit;

struct CorridorPlanVisualization::Data
{
    corridors::Plan plan;
    corridors::Corridor selected_corridor;
    bool has_corridor;

    envire::ElevationGrid const* grid;
    double offset;
    std::vector<osg::Vec4> colors;
    double alpha;

    Data()
        : has_corridor(true), grid(0), offset(0.0), alpha(0.5) {}
};

CorridorPlanVisualization::CorridorPlanVisualization()
    : p(new Data())
{
}

CorridorPlanVisualization::~CorridorPlanVisualization()
{ delete p; }

void CorridorPlanVisualization::setElevationGrid(envire::ElevationGrid const* heights, double offset)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->grid   = heights;
    p->offset = offset;
    setDirty();
}

double CorridorPlanVisualization::getElevation(Eigen::Vector3d const& point) const
{
    if (p->grid)
        return p->grid->get(point.x(), point.y()) + p->offset;
    else
        return 0;
}

void CorridorPlanVisualization::createCurveNode(osg::Geode* geode, base::geometry::Spline<3>& curve, osg::Vec4 const& color, double z_offset)
{
    if (curve.isEmpty())
        return;

    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

    double start = curve.getStartParam();
    double step  = (curve.getEndParam() - curve.getStartParam()) / curve.getCurveLength() / 10;
    double last_z;

    Eigen::Vector3d last_p;
    for (double t = start; t < curve.getEndParam(); t += step)
    {
        Eigen::Vector3d p = curve.getPoint(t);
        double z = getElevation(p) + z_offset;
        if (t == start)
        {
            last_p = p;
            last_z = z;
        }
        else
        {
            z = 0.5 * last_z + 0.5 * z;
            last_z = z;
        }

        points->push_back(osg::Vec3(p.x(), p.y(), p.z()));
    }

    osg::DrawArrays* painter =
        new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, points->size());

    osg::Geometry* geom = new osg::Geometry;

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(color);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(painter);
    geom->setVertexArray(points);
    geode->addDrawable(geom);
}

void CorridorPlanVisualization::createCorridorNode(osg::Geode* geode, corridors::Corridor& c, osg::Vec4 const& color, double z_offset)
{
    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

    double max_length = 0;
    for (int i = 0; i < 2; ++i)
    {
        base::geometry::Spline<3>& spline = c.boundary_curves[i];
        if (spline.isEmpty())
        {
            return;
        }
        double length = fabs(spline.getCurveLength());
        if (max_length < length)
           max_length = length;
    }

    double curve_starts[2];
    double curve_steps[2];
    for (int i = 0; i < 2; ++i)
    {
        base::geometry::Spline<3>& spline = c.boundary_curves[i];
        double start  = spline.getStartParam();
        double end    = spline.getEndParam();
        curve_starts[i] = start;
        curve_steps[i] = (end - start) / max_length / 100;
    }

    double old_z[2];
    for (int i = 0; i < max_length * 100 - 1; ++i)
    {
        for (int curve = 0; curve < 2; ++curve)
        {
            double t = curve_starts[curve] + curve_steps[curve] * i;
            Eigen::Vector3d p = c.boundary_curves[curve].getPoint(t);
            double z = getElevation(p) + z_offset;
            if (i == 0)
                old_z[curve] = z;
            else
                z = 0.5 * old_z[curve] + 0.5 * z;
            old_z[curve] = z;

            points->push_back(osg::Vec3(p.x(), p.y(), z));
        }
    }

    osg::DrawArrays* painter =
        new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP, 0, points->size());

    osg::Geometry* geom = new osg::Geometry;

    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(color);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(painter);
    geom->setVertexArray(points);
    geode->addDrawable(geom);
}

void CorridorPlanVisualization::computeColors(int size)
{
    p->colors.clear();

    size_t gradient_size = (size + 5) / 6;
    float base = 0.3;
    float step = (1.0 - base) / gradient_size;
    for (size_t i = 0; i < gradient_size; ++i)
    {
        p->colors.push_back(osg::Vec4(base,            base + step * i, base + step * i, p->alpha));
        p->colors.push_back(osg::Vec4(base + step * i, base,            base + step * i, p->alpha));
        p->colors.push_back(osg::Vec4(base + step * i, base + step * i, base,            p->alpha));

        p->colors.push_back(osg::Vec4(1.0,             base + step * i, base + step * i, p->alpha));
        p->colors.push_back(osg::Vec4(base + step * i, 1.0,             base + step * i, p->alpha));
        p->colors.push_back(osg::Vec4(base + step * i, base + step * i, 1.0,             p->alpha));
    }
}

osg::Vec4 CorridorPlanVisualization::getColor(int i) const
{
    return p->colors[i];
}

osg::ref_ptr<osg::Node> CorridorPlanVisualization::createMainNode()
{
    return new osg::Geode();
}

void CorridorPlanVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = dynamic_cast<osg::Geode*>(node);

    int corridor_count = p->plan.corridors.size();

    // Clear the geode
    geode->removeDrawables(0, geode->getDrawableList().size());
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    // Update the color set
    computeColors(corridor_count);

    // Create objects for each of the plan's corridors
    for (int i = 0; i < corridor_count; ++i)
    {
        corridors::Corridor& c = p->plan.corridors[i];
        createCorridorNode(geode, c, getColor(i), 0);
    }

    // If we have a selected corridor, also display it
    if (p->has_corridor)
    {
        createCurveNode(geode, p->selected_corridor.median_curve, osg::Vec4(1.0, 1.0, 1.0, 1.0), 0.5);
        createCurveNode(geode, p->selected_corridor.boundary_curves[0], osg::Vec4(0.75, 0.5, 0.25, 1.0), 0.5);
        createCurveNode(geode, p->selected_corridor.boundary_curves[1], osg::Vec4(0.25, 0.5, 0.75, 1.0), 0.5);
    }
}

void CorridorPlanVisualization::setAlpha(double value)
{
    p->alpha = value;
    setDirty();
}

void CorridorPlanVisualization::updateDataIntern(corridors::Plan const& plan)
{
    p->plan = plan;
}

void CorridorPlanVisualization::updateDataIntern(corridors::Corridor const& selected_corridor)
{
    p->has_corridor = true;
    p->selected_corridor = selected_corridor;
}

