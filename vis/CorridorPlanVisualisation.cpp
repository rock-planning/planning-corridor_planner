#include "CorridorPlanVisualisation.h"
#include "corridors.hh"

#include <osg/Geometry>
#include <osg/Geode>

#include <envire/maps/Grids.hpp>

using namespace corridor_planner;

struct PlanVisualisation::Data
{
    corridors::Plan plan;
    envire::ElevationGrid const* grid;
    double offset;
    std::vector<osg::Vec4> colors;
    double alpha;
    Data()
        : grid(0), offset(0.0), alpha(1.0) {}
};

PlanVisualisation::PlanVisualisation()
    : p(new Data())
{
    setMainNode(new osg::Geode);
}

PlanVisualisation::~PlanVisualisation()
{ delete p; }

void PlanVisualisation::setElevationGrid(envire::ElevationGrid const* heights, double offset)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->grid   = heights;
    p->offset = offset;
    setDirty();
}

osg::Geode* PlanVisualisation::getMainNode() const
{ return dynamic_cast<osg::Geode*>(enview::DataNode<corridors::Plan>::getMainNode()); }

void PlanVisualisation::sampleSpline(base::geometry::Spline<3>& spline, osg::Vec3Array& points)
{
    double start = spline.getStartParam();
    double end = spline.getEndParam();
    double length = fabs(spline.getCurveLength());
    double step = (end - start) / length / 100;
    for (double t = start; t < end; t += step)
    {
        Eigen::Vector3d point = spline.getPoint(t);
        double z = 0;
        if (p->grid)
            z = p->grid->get(point.x(), point.y()) + p->offset;
        points.push_back(osg::Vec3(point.x(), point.y(), z));

        // For all points except the starting point, add the points twice as
        // DrawArrays expects (start_point, end_point) pairs
        if (points.size() > 1)
            points.push_back(osg::Vec3(point.x(), point.y(), z));
    }
}

void PlanVisualisation::createCorridorNode(osg::Geode* geode, corridors::Corridor& c, osg::Vec4 const& color)
{
    osg::Vec4Array* colors = new osg::Vec4Array();
    colors->push_back(color);

    for (int i = 0; i < 2; ++i)
    {
        osg::Vec3Array* points = new osg::Vec3Array;
        sampleSpline(c.boundary_curves[i], *points);

        osg::DrawArrays* painter =
            new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, points->size());

        osg::Geometry*  geom   = new osg::Geometry;
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom->addPrimitiveSet(painter);
        geom->setVertexArray(points);
        geode->addDrawable(geom);
    }
}

void PlanVisualisation::computeColors(int size)
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

osg::Vec4 PlanVisualisation::getColor(int i) const
{
    return p->colors[i];
}

void PlanVisualisation::operatorIntern ( osg::Node* node, osg::NodeVisitor* nv )
{
    std::cerr << "updating corridor display" << std::endl;
    int corridor_count = p->plan.corridors.size();

    // Clear the geode
    osg::Geode* geode = dynamic_cast<osg::Geode*>(getMainNode());
    geode->removeDrawables(0, geode->getDrawableList().size());
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    // Update the color set
    computeColors(corridor_count);

    for (int i = 0; i < corridor_count; ++i)
    {
        corridors::Corridor& c = p->plan.corridors[i];
        createCorridorNode(geode, c, getColor(i));
    }
    std::cerr << "DONE" << std::endl;
}

void PlanVisualisation::setAlpha(double value)
{
    p->alpha = value;
    setDirty();
}

void PlanVisualisation::updateDataIntern(corridors::Plan const& plan)
{
    p->plan = plan;
}

