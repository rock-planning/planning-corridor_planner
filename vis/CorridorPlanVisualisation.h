#ifndef PLAN_VISUALISATION_H
#define PLAN_VISUALISATION_H

#include <boost/noncopyable.hpp>
#include <enview/DataNode.h>
#include <osg/Geometry>
#include <envire/maps/Grids.hpp>
#include <base/geometry/spline.h>

namespace corridors
{
    class Plan;
    class Corridor;
}

namespace corridor_planner
{
    class PlanVisualisation
        : public enview::DataNode<corridors::Plan>
        , boost::noncopyable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PlanVisualisation();
        ~PlanVisualisation();
        osg::Geode* getMainNode() const;

        void computeColors(int size);
        osg::Vec4 getColor(int i) const;

        void setElevationGrid(envire::ElevationGrid const* heights, double offset);
        
    protected:
        virtual void operatorIntern(osg::Node* node, osg::NodeVisitor* nv);
        virtual void updateDataIntern(corridors::Plan const& plan);
        
    private:
        void sampleSpline(base::geometry::Spline<3>& spline, osg::Vec3Array& points);
        osg::Geometry* createCorridorNode(corridors::Corridor& c, osg::Vec4 const& color);

        struct Data;
        Data* p;
    };
}
#endif
