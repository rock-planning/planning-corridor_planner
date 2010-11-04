#include "CorridorPlanVisualisation.h"
#include "corridors.hh"

#include <osg/Geometry>
#include <osg/Geode>

#include <envire/maps/Grids.hpp>

using namespace corridor_planner;

struct PlanVisualisation::Data
{
    corridors::Plan plan;
    envire::ElevationGrid& grid;
    Data(envire::ElevationGrid& grid)
        : grid(grid) {}
};

PlanVisualisation::PlanVisualisation(envire::ElevationGrid& heights)
    : p(new Data(heights))
{
    setMainNode(new osg::Geode);
}

PlanVisualisation::~PlanVisualisation()
{ delete p; }

void PlanVisualisation::setElevationGrid(envire::ElevationGrid const* heights)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->grid = heights;
    setDirty();
}

osg::Geode* PlanVisualisation::getMainNode() const
{ return dynamic_cast<osg::Geode*>(enview::DataNode<corridors::Plan>::getMainNode()); }

void PlanVisualisation::sampleSpline(base::geometry::Spline<3>& spline, osg::Vec3Array& points)
{
    double step = spline.getUnitParameter() / 10;
    for (double t = spline.getStartParam(); t < spline.getEndParam(); t += step)
    {
        Eigen::Vector3d point = spline.getPoint(t);
        double z = p->grid.get(point.x(), point.y());
        points.push_back(osg::Vec3(point.x(), point.y(), z));

        // For all points except the starting point, add the points twice as
        // DrawArrays expects (start_point, end_point) pairs
        if (points.size() > 1)
            points.push_back(osg::Vec3(point.x(), point.y(), z));
    }
}

osg::Geometry* PlanVisualisation::createCorridorNode(corridors::Corridor& c, osg::Vec4 const& color)
{
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(color);

    osg::Vec3Array* points = new osg::Vec3Array;
    sampleSpline(c.boundary_curves[0], *points);
    sampleSpline(c.boundary_curves[1], *points);

    osg::DrawArrays* painter =
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, points->size());

    osg::Geometry*  geom   = new osg::Geometry;
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->addPrimitiveSet(painter);
    geom->setVertexArray(points);
    return geom;
}

void PlanVisualisation::computeColors(int size)
{
}

osg::Vec4 PlanVisualisation::getColor(int i) const
{
    return osg::Vec4(1.0, 1.0, 1.0, 1.0);
}

void PlanVisualisation::operatorIntern ( osg::Node* node, osg::NodeVisitor* nv )
{
    int corridor_count = p->plan.corridors.size();

    // Clear the geode
    osg::Geode* geode = dynamic_cast<osg::Geode*>(getMainNode());
    geode->removeDrawables(0, geode->getDrawableList().size());

    // Update the color set
    computeColors(corridor_count);

    for (int i = 0; i < corridor_count; ++i)
    {
        corridors::Corridor& c = p->plan.corridors[i];
        osg::Geometry* geom = createCorridorNode(c, getColor(i));
        geode->addDrawable(geom);
    }

}

void PlanVisualisation::updateDataIntern(corridors::Plan const& plan)
{
    p->plan = plan;
}

