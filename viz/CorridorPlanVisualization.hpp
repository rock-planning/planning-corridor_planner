#ifndef PLAN_VISUALISATION_H
#define PLAN_VISUALISATION_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <osg/Geometry>

#include <envire/maps/Grids.hpp>
#include <base/geometry/spline.h>

namespace corridors
{
    class Plan;
    class Corridor;
}

namespace vizkit
{
    class CorridorPlanVisualization
        : public vizkit::VizPlugin<corridors::Plan>
        , boost::noncopyable
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CorridorPlanVisualization();
        ~CorridorPlanVisualization();

        void computeColors(int size);
        osg::Vec4 getColor(int i) const;

        void setAlpha(double value);
        void setMLS(std::string const& path);
        void setZOffset(double value);
        void displayCorridor(corridors::Corridor const& selected_corridor);
        void clearCorridors(double);
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(corridors::Plan const& plan);
        
    private:
        double getElevation(Eigen::Vector3d const& point) const;
        void createCurveNode(osg::Geode* geode, base::geometry::Spline<3>& curve, osg::Vec4 const& color, double z_offset);
        void createCorridorNode(osg::Geode* geode, corridors::Corridor& c, osg::Vec4 const& color, double z_offset);

        struct Data;
        Data* p;
    };
}
#endif
