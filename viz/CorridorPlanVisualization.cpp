#include "CorridorPlanVisualization.hpp"
#include "corridor_planner/corridors.hh"

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>

#include <envire/maps/MLSGrid.hpp>

using namespace vizkit;
using envire::MLSGrid;

struct CorridorPlanVisualization::Data
{
    corridors::Plan plan;
    bool plan_annotations;
    typedef std::list<corridors::Corridor> Corridors;
    Corridors corridors;

    envire::Environment* environment;
    envire::MLSGrid::Ptr mls;
    double offset;
    std::vector<osg::Vec4> colors;
    double alpha;
    std::string displayed_annotation;
    std::vector<osg::Vec4> annotation_colors;

    Data()
        : plan_annotations(false), environment(0), offset(0.1), alpha(0.5) {}
};

CorridorPlanVisualization::CorridorPlanVisualization()
    : p(new Data())
{
    VizPluginRubyAdapter(CorridorPlanVisualization, corridors::Plan, Plan);
    VizPluginRubyConfig(CorridorPlanVisualization, double, setAlpha);
    VizPluginRubyConfig(CorridorPlanVisualization, double, setZOffset);
    VizPluginRubyConfig(CorridorPlanVisualization, std::string, setMLS);
    VizPluginRubyConfig(CorridorPlanVisualization, corridors::Corridor, displayCorridor);
    VizPluginRubyConfig(CorridorPlanVisualization, double, clearCorridors);
    VizPluginRubyConfig(CorridorPlanVisualization, std::string, setDisplayedAnnotation);
    VizPluginRubyConfig(CorridorPlanVisualization, bool, displayAnnotationsOnPlan);
}

CorridorPlanVisualization::~CorridorPlanVisualization()
{ delete p; }

void CorridorPlanVisualization::setMLS(std::string const& path)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    if (p->environment)
        delete p->environment;
    p->environment = envire::Environment::unserialize(path);
    p->mls   = p->environment->getItem<envire::MLSGrid>();
    std::cout << "loaded MLS from " << path << std::endl;
    setDirty();
}

void CorridorPlanVisualization::setZOffset(double offset)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->offset = offset;
    setDirty();
}

void CorridorPlanVisualization::setDisplayedAnnotation(std::string const& name)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->displayed_annotation = name;
    setDirty();
}

double CorridorPlanVisualization::getElevation(Eigen::Vector3d const& point) const
{
    if (p->mls)
    {
        size_t m, n;
        p->mls->toGrid(point, m, n, p->environment->getRootNode());
        MLSGrid::const_iterator it = std::max_element(p->mls->beginCell(m, n), p->mls->endCell());
        if (it != p->mls->endCell())
            return it->mean;
        else
            return 0;
    }
    else
        return 0;
}

void CorridorPlanVisualization::createCurveNode(osg::Geode* geode, base::geometry::Spline<3>& curve,
        osg::Vec4 color, double z_offset, double line_width)
{
    std::vector<corridors::Corridor::AnnotatedSegment> annotations;
    std::vector<osg::Vec4> colors;
    colors.push_back(color);
    createCurveNode(geode, curve, annotations, colors, z_offset, line_width);
}


void CorridorPlanVisualization::createCurveNode(osg::Geode* geode, base::geometry::Spline<3>& curve,
        std::vector<corridors::Corridor::AnnotatedSegment> annotations,
        std::vector<osg::Vec4> colors, double z_offset, double line_width)
{
    if (curve.isEmpty())
        return;

    int min_annotation = std::numeric_limits<int>::max();
    for (unsigned int i = 0; i < annotations.size(); ++i)
        min_annotation = std::min(min_annotation, annotations[i].symbol);

    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;

    osg::Vec4Array* vertex_colors = new osg::Vec4Array();
    osg::Vec4 color;

    double last_z;
    std::vector<double> parameters;
    std::vector<base::Vector3d> curve_points = curve.sample(0.05, &parameters);

    for (unsigned int point_idx = 0; point_idx < curve_points.size(); ++point_idx)
    {
        double t = parameters[point_idx];

        osg::Vec4 color = colors[0];
        while (!annotations.empty() && annotations.front().end <= t)
            annotations.erase(annotations.begin());

        if (!annotations.empty())
        {
            if (annotations.front().start > t)
                color = colors[0];
            else
                color = colors[1 + annotations.front().symbol - min_annotation];
        }


        Eigen::Vector3d p = curve_points[point_idx];
        double z = getElevation(p) + z_offset;
        if (point_idx == 0)
            last_z = z;
        else
        {
            z = 0.5 * last_z + 0.5 * z;
            last_z = z;
        }

        points->push_back(osg::Vec3(p.x(), p.y(), z));
        vertex_colors->push_back(color);
    }

    osg::DrawArrays* painter =
        new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, points->size());

    osg::Geometry* geom = new osg::Geometry;
    geom->setColorArray(vertex_colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(painter);
    geom->setVertexArray(points);

    if (line_width != 0)
    {
        osg::StateSet* stateset = new osg::StateSet;
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(line_width);
        stateset->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
        geom->setStateSet(stateset);
    }

    geode->addDrawable(geom);
}

void CorridorPlanVisualization::createCorridorNode(osg::Geode* geode, corridors::Corridor& c, int annotation_idx, osg::Vec4 const& color, double z_offset)
{
    if (annotation_idx == -1 || !p->plan_annotations)
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

    base::geometry::Spline<3> start_line(0.1, 2);
    std::vector<base::Vector3d> points;
    points.push_back(c.boundary_curves[0].getStartPoint());
    points.push_back(c.median_curve.getStartPoint());
    if (points[0] == points[1]) points.pop_back();
    points.push_back(c.boundary_curves[1].getStartPoint());
    if (points[0] == points[1]) points.pop_back();
    start_line.interpolate(points);
    createCurveNode(geode, start_line, osg::Vec4(1, 1, 1, 1), p->offset + 0.1, 5);

    if (annotation_idx != -1 && p->plan_annotations)
    {
        createCurveNode(geode, c.median_curve, c.annotations[0][annotation_idx], p->annotation_colors, p->offset + 0.1, 10);
        createCurveNode(geode, c.boundary_curves[0], c.annotations[1][annotation_idx], p->annotation_colors, p->offset + 0.1, 5);
        createCurveNode(geode, c.boundary_curves[1], c.annotations[2][annotation_idx], p->annotation_colors, p->offset + 0.1, 5);
    }
    else
    {
        createCurveNode(geode, c.median_curve, osg::Vec4(1.0, 1.0, 1.0, 1.0), p->offset + 0.1, 10);
        createCurveNode(geode, c.boundary_curves[0], osg::Vec4(0.75, 0.5, 0.25, 1.0), p->offset + 0.1, 5);
        createCurveNode(geode, c.boundary_curves[1], osg::Vec4(0.25, 0.5, 0.75, 1.0), p->offset + 0.1, 5);
    }
}

void CorridorPlanVisualization::computeColors(std::vector<osg::Vec4>& colors, int size)
{
    colors.clear();

    size_t gradient_size = (size + 5) / 6 + 1;
    float base = 0.3;
    float step = (1.0 - base) / gradient_size;
    colors.push_back(osg::Vec4(base, base, base, p->alpha));
    for (size_t i = 0; i < gradient_size; ++i)
    {
        if (i != 0)
            colors.push_back(osg::Vec4(base,            base + step * i, base + step * i, p->alpha));
        colors.push_back(osg::Vec4(1.0,             base + step * i, base + step * i, p->alpha));
        if (i != 0)
            colors.push_back(osg::Vec4(base + step * i, base,            base + step * i, p->alpha));
        colors.push_back(osg::Vec4(base + step * i, 1.0,             base + step * i, p->alpha));
        if (i != 0)
            colors.push_back(osg::Vec4(base + step * i, base + step * i, base,            p->alpha));
        colors.push_back(osg::Vec4(base + step * i, base + step * i, 1.0,             p->alpha));
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
    computeColors(p->colors, corridor_count);

    int annotation_idx = -1;
    if (!p->displayed_annotation.empty())
    {
        annotation_idx = p->plan.findAnnotationIndex(p->displayed_annotation);
        if (annotation_idx == -1)
        {
            std::cerr << "cannot find required annotation " << p->displayed_annotation << std::endl;
            std::cerr << "known annotations in this plan:" << std::endl;
            for (unsigned int i = 0; i < p->plan.annotation_symbols.size(); ++i)
                std::cerr << "  " << p->plan.annotation_symbols[i] << std::endl;
        }
    }

    if (annotation_idx != -1)
    {
        int min_annotation = std::numeric_limits<int>::max(), max_annotation = std::numeric_limits<int>::min();
        for (int corridor_idx = 0; corridor_idx < corridor_count; ++corridor_idx)
        {
            corridors::Corridor& c = p->plan.corridors[corridor_idx];
            for (int curve_idx = 0; curve_idx < 3; ++curve_idx)
            {
                corridors::Corridor::Annotations& annotations = c.annotations[curve_idx][annotation_idx];
                for (unsigned int i = 0; i < annotations.size(); ++i)
                {
                    min_annotation = std::min(annotations[i].symbol, min_annotation);
                    max_annotation = std::max(annotations[i].symbol, max_annotation);
                }
            }
        }

        if (min_annotation < max_annotation)
            computeColors(p->annotation_colors, max_annotation - min_annotation + 2);
        else
            computeColors(p->annotation_colors, 2);
    }

    // Create objects for each of the plan's corridors
    for (int i = 0; i < corridor_count; ++i)
    {
        corridors::Corridor& c = p->plan.corridors[i];
        createCorridorNode(geode, c, annotation_idx, getColor(i), p->offset);
    }

    // If we have a selected corridor, also display it
    for (CorridorPlanVisualization::Data::Corridors::iterator it = p->corridors.begin(); it != p->corridors.end(); ++it)
    {
        createCorridorNode(geode, *it, annotation_idx, osg::Vec4(0, 0, 0, 0), p->offset + 0.1);
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
void CorridorPlanVisualization::displayAnnotationsOnPlan(bool enable)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->plan_annotations = enable;
    setDirty();
}

void CorridorPlanVisualization::clearCorridors(double)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->corridors.clear();
    setDirty();
}

void CorridorPlanVisualization::displayCorridor(corridors::Corridor const& selected_corridor)
{ boost::mutex::scoped_lock lockit(this->updateMutex);
    p->corridors.push_back(selected_corridor);
    setDirty();
}

VizkitQtPlugin(CorridorPlanVisualization);

