#include "CorridorPlanVisualization.hpp"
#include "corridors.hh"

#include <osg/Geometry>
#include <osg/Geode>

#include <envire/maps/Grids.hpp>

using namespace corridor_planner;

struct PlanVisualization::Data
{
    corridors::Plan plan;
    envire::ElevationGrid const* grid;
    double offset;
    std::vector<osg::Vec4> colors;
    double alpha;
    Data()
        : grid(0), offset(0.0), alpha(0.5) {}
};

PlanVisualization::PlanVisualization()
    : p(new Data())
{
}

PlanVisualization::~PlanVisualization()
{ delete p; }

void PlanVisualization::setElevationGrid(envire::ElevationGrid const* heights, double offset)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->grid   = heights;
    p->offset = offset;
    setDirty();
}

double PlanVisualization::getElevation(Eigen::Vector3d const& point) const
{
    if (p->grid)
        return p->grid->get(point.x(), point.y()) + p->offset;
    else
        return 0;
}

void PlanVisualization::createCorridorNode(osg::Geode* geode, corridors::Corridor& c, osg::Vec4 const& color)
{
    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

    double max_length = 0;
    for (int i = 0; i < 2; ++i)
    {
        base::geometry::Spline<3>& spline = c.boundary_curves[i];
        if (spline.isEmpty())
        {
            std::cerr << "  empty boundary" << std::endl;
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
            std::cerr << i << " " << curve_steps[curve] << " " << t << " " << c.boundary_curves[curve].getEndParam() << std::endl;
            Eigen::Vector3d p = c.boundary_curves[curve].getPoint(t);
            double z = getElevation(p);
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

void PlanVisualization::computeColors(int size)
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

osg::Vec4 PlanVisualization::getColor(int i) const
{
    return p->colors[i];
}

osg::ref_ptr<osg::Node> PlanVisualization::createMainNode()
{
    return new osg::Geode();
}

void PlanVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = dynamic_cast<osg::Geode*>(node);

    int corridor_count = p->plan.corridors.size();
    std::cerr << "displaying " << corridor_count << " corridors" << std::endl;

    // Clear the geode
    geode->removeDrawables(0, geode->getDrawableList().size());
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    // Update the color set
    computeColors(corridor_count);

    for (int i = 0; i < corridor_count; ++i)
    {
        corridors::Corridor& c = p->plan.corridors[i];
        std::cerr << "handling corridor " << i << std::endl;
        createCorridorNode(geode, c, getColor(i));
    }
    std::cerr << "DONE" << std::endl;
}

void PlanVisualization::setAlpha(double value)
{
    p->alpha = value;
    setDirty();
}

void PlanVisualization::updateDataIntern(corridors::Plan const& plan)
{
    std::cerr << "new plan with " << plan.corridors.size() << " corridors" << std::endl;
    p->plan = plan;
}

