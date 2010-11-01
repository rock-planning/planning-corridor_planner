#ifndef PLAN_VISUALISATION_H
#define PLAN_VISUALISATION_H

#include <boost/noncopyable.hpp>
#include <enview/DataNode.h>
#include <osg/Geometry>
#include <envire/maps/Grids.hpp>
#include <base/geometry/spline.h>

namespace corridor_planner
{
    class Plan;
    class Corridor;
    class PlanVisualisation
        : public enview::DataNode<corridor_planner::Plan>
        , boost::noncopyable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PlanVisualisation(envire::ElevationGrid& heights);
        ~PlanVisualisation();
        osg::Geode* getMainNode() const;

        void computeColors(int size);
        osg::Vec4 getColor(int i) const;
        
    protected:
        virtual void operatorIntern(osg::Node* node, osg::NodeVisitor* nv);
        virtual void updateDataIntern(corridor_planner::Plan const& plan);
        
    private:
        void sampleSpline(base::geometry::Spline<3>& spline, osg::Vec3Array& points);
        osg::Geometry* createCorridorNode(Corridor& c, osg::Vec4 const& color);

        struct Data;
        Data* p;
    };
}
#endif
