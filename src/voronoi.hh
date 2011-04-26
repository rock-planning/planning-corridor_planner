#ifndef NAV_VORONOI_HH
#define NAV_VORONOI_HH
#include <list>
#include <map>
#include <vector>
#include <iosfwd>
#include <boost/tuple/tuple.hpp>

#include <base/geometry/spline.h>
#include <nav_graph_search/point.hpp>
#include <nav_graph_search/geometry.hpp>
#include <nav_graph_search/grid_graph.hpp>

namespace corridor_planner
{
    typedef nav_graph_search::PointID PointID;
    typedef nav_graph_search::PointSet PointSet;
    typedef nav_graph_search::BoundingBox BoundingBox;
    typedef nav_graph_search::PointVector PointVector;

    typedef nav_graph_search::GridGraph GridGraph;

    /** This method updates the set of point sets \c sets by adding \c p so
     * that:
     * <ul>
     *  <li> the points in each set of \c sets form a connected set
     *  <li> no two sets are connected
     * </ul>
     *
     * @returns the element of \c sets which now contains \c p
     */
    std::list<PointSet>::iterator updateConnectedSets(std::list<PointSet>& sets, PointID p);

    /** Descriptor for one point of the voronoi diagram. This descriptor
     * maintains the association between the point and the associated border
     * points
     */
    struct VoronoiPoint
    {
        BoundingBox bbox;
        typedef std::list< PointVector > BorderList;

        PointID center;

        nav_graph_search::Point<float> tangent;

        /** VoronoiPoint maintains a list of borders, each borders being a set of
         * adjacent points. This means that:
         * <ul>
         * <li> in each element of \c borders, the point form a connected graph
         * <li> there is no connection between two different elements of \c * borders
         * </ul>
         *
         * These contraints are maintained by addBorderPoint and mergeBorders
         */
        BorderList borders;

        /** The distance between the point and the borders. Note that there is
         * one unique distance by construction
         */
        int width;

        VoronoiPoint() : width(0)  {}
        bool operator == (VoronoiPoint const& other) const;

        nav_graph_search::Point<float> direction() const;
        bool isSingleton() const;

        void offset(PointID const& v);

        template<typename _It>
        void addBorder(_It begin, _It end)
        {
            borders.push_back(std::vector<PointID>());
            std::vector<PointID>& new_border = borders.back();
            for (_It it = begin; it != end; ++it)
                new_border.push_back(*it);
        }

        /** Add \c p to the borders. See borders for more details */
        void addBorderPoint(PointID const& p);
        /** Merge the borders of \c p into the ones of \c this */
        void mergeBorders(VoronoiPoint const& p);

        /** Returns true if for each border of \c p there is at least one
         * touching border in \c this */
        bool isBorderAdjacent(VoronoiPoint const& p) const;
        bool isBorderAdjacent(PointID const& p) const;

        typedef VoronoiPoint::BorderList::iterator border_iterator;
        typedef VoronoiPoint::BorderList::const_iterator border_const_iterator;
        border_const_iterator findAdjacentBorder(PointID const& p) const;
    };
    inline PointID get_point(VoronoiPoint const& v)
    { return v.center; }

    std::ostream& operator << (std::ostream& io, VoronoiPoint const& p);

    /** In the plan representation, a corridor is a "tunnel" that the agent will
     * traverse. The navigation constraint is that, once the agent got in at one
     * side of the tunnel, it has to go out at the other side.
     *
     * There are two point of view: the geometrical and the cost one.
     *
     * The geometrical point of view is based on the voronoi line
     * (Corridor.voronoi). From that point of view,, voronoi.front() is called
     * the front point while voronoi.back() is called the back point.
     *
     * The cost point of view is based on whether the corridor descends the cost
     * gradient or ascends it. From that point of view, the front point is the
     * point of higher cost and the back point the point of lower cost (remember
     * that we descend the cost function to go nearer to the goal).
     *
     * The mapping between the geometrical and cost point of view is saved in
     * the Corridor::end_types array. end_types[0] is the cost representation of
     * the geometrical front point and end_types[1] the one of the geometrical
     * back point.
     *
     * I.e. if end_types[0] == ENDPOINT_BACK, it means that descending in the
     * geometrical order will ascend the cost order.
     */
    class Corridor
    {
        /** Updates the width curve. This is called by updateCurves internally,
         * to generate a 1-D curve at which, for each parameter value from the
         * median line, provides the width at this parameter.
         */
        void updateWidthCurve();

        /** Clears up the ordering of points in a line, by applying a graph
         * search method. The method searches the longest branches in a DFS
         * which are not adjacent to other branches, and modifies +line+ to
         * return this "line"
         */
        void fixLineOrdering(GridGraph& graph, std::list<PointID>& line);

    public:
        std::list<VoronoiPoint> voronoi;
        std::list<PointID> boundaries[2];

        base::geometry::Spline<3> median_curve;
        base::geometry::Spline<3> boundary_curves[2];

        base::geometry::Spline<1> width_curve;
        float min_width, max_width;

        static const int ENDPOINT_UNKNOWN = 0;
        static const int ENDPOINT_FRONT   = 1;
        static const int ENDPOINT_BACK    = 2;
        static const int ENDPOINT_BIDIR   = 3;

	int   end_types[2];
        /** The cost delta when traversing this corridor in the geometrical
         * direction
         *
         * See Corridor::getCostDelta();
         */
        float dcost;
	bool  bidirectional;

        BoundingBox bbox;
        BoundingBox median_bbox;

        std::string name;

        /** Returns true if this corridor contains only one point
         */
        bool isSingleton() const
        { return voronoi.size() == 1; }

        struct ConnectionDescriptor
        {
            bool this_side; // true for back() and false for front()
            int  target_idx;
            bool target_side; // true for back() and false for front()
            ConnectionDescriptor(bool side, int target_idx, bool target_side)
                : this_side(side), target_idx(target_idx), target_side(target_side)
            {}
        };

        typedef std::list<ConnectionDescriptor> Connections;
        typedef Connections::iterator connection_iterator;
        typedef Connections::const_iterator const_connection_iterator;
        Connections connections;

	Corridor();

        void swap(Corridor& other);
        void update();

        /** Returns true if \c p is contained in this corridor */
        bool contains(PointID const& p) const;

        /** Returns true if \c p is either contained in, or touches the border of,
         * the corridor
         */
        bool isNeighbour(PointID const& p) const;

        /** Returns true iff +p+ is neighbouring the median line
         */
        bool isMedianNeighbour(PointID const& p) const;

        void clear();
        bool operator == (Corridor const& other) const;

        /** Returns the side of the corridot that is closest to the given point.
         * Returns true for front and false for back.
         */
	bool findSideOf(PointID const& p) const;

        /** Inverts the geometrical direction of this corridor
         *
         * NOTE: this should not be used as-is, as the connections are left
         * unchanged. Instead, one should use Plan::reverse()
         */
        void reverse();

        void push_back(PointID const& p, VoronoiPoint const& descriptor);
        void push_back(VoronoiPoint const& descriptor);

        /** Merge \c corridor into this one, updating its border, median line
         * and bounding box
         */
        void concat(Corridor& corridor);

        /** Returns the set of corridors the specified side is connected to */
        std::set<int> connectivity(bool side) const;

        /** Returns the set of corridors this one is connected to */
        std::set<int> connectivity() const;

        /** Returns true if there is at least one connection from this corridor
         * to \c other_corridor
         */
        bool isConnectedTo(int other_corridor) const;

        /** Registers a new connection from this corridor to +target_idx+.
         * +side+ and +target_side+ specify at which side of the corridor the
         * connection is attached. \c means front and \c true means back.
         */
        void addConnection(bool side, int target_idx, bool target_side);

        /** Remove the specified connection */
        void removeConnection(int target_idx, bool target_side);

        /** Removes all connections that point to \c other_corridor. It does
         * not remove them on \c other_corridor
         */
        void removeConnectionsTo(int other_corridor);

        /** Change the connection definition so that all connections previously
         * pointing to +prev_idx+ are now pointing to +new_idx+
         */
        void moveConnections(size_t prev_idx, size_t new_idx);
        
        /** Copy all the connections that come from the specified side of the
         * corridor onto the other corridor side
         */
        void copyOutgoingConnections(bool this_side, Corridor& other_corridor, bool other_side) const;

        /** Copy all outgoing connections from +other_corridor+ onto this
         * corridor.
         */
        void moveOutgoingConnections(Corridor& other_corridor);

        size_t size() const { return voronoi.size(); }

        VoronoiPoint& front() { return voronoi.front(); }
        VoronoiPoint& back()  { return voronoi.back(); }
        VoronoiPoint const& front() const { return voronoi.front(); }
        VoronoiPoint const& back() const  { return voronoi.back(); }

        PointID frontPoint() const { return voronoi.front().center; }
        PointID backPoint() const { return voronoi.back().center; }
        PointID getEndpoint(bool side) const
        { return side ? backPoint() : frontPoint(); }
        Eigen::Vector3d getMedianCurveEndpoint(bool side) const
        { return side ? median_curve.getEndPoint() : median_curve.getStartPoint(); }

        PointID getBoundaryEndpoint(int boundary_idx, bool side) const
        {
            return side ? boundaries[boundary_idx].back() :
                boundaries[boundary_idx].front();
        }
        Eigen::Vector3d getBoundaryCurveEndpoint(int boundary_idx, bool side) const
        { return side ? boundary_curves[boundary_idx].getEndPoint() : boundary_curves[boundary_idx].getStartPoint(); }

        typedef std::list<VoronoiPoint>::const_iterator voronoi_const_iterator;
        typedef std::list<VoronoiPoint>::iterator voronoi_iterator;
        voronoi_iterator begin() { return voronoi.begin(); }
        voronoi_iterator end()   { return voronoi.end(); }
        voronoi_const_iterator begin() const { return voronoi.begin(); }
        voronoi_const_iterator end() const   { return voronoi.end(); }

        bool isBorderAdjacent(VoronoiPoint const& p) const;

        /** Returns a point of the median line which is adjacent to \c p */
        PointID adjacentEndpoint(PointID const& p) const;

        voronoi_iterator findMedianPoint(PointID const& p);
        voronoi_const_iterator findMedianPoint(PointID const& p) const;

        /** Returns an interator on the point of the median nearest to \c p */
        voronoi_iterator findNearestMedian(PointID const& p);
        voronoi_const_iterator findNearestMedian(PointID const& p) const;

        /** Returns true if the corridor invariants are met, and false otherwise
         */
        bool checkConsistency() const;

        /** Create a singleton corridor which includes the provided point
         */
        static Corridor singleton(PointID const& p, std::string const& name = "");

        /** Creates the spline that represents the boundary and the voronoi
         * median line. To make the B-splines usable, the pixel lines are
         * smoothed by a IIR filter with the provided discount factor
         */
        void updateCurves(double discount_factor = 0.5);

        /** Returns the cost of traversing this corridor, based on which way is
         * taken.
         */
        float getCostDelta(bool traverse_backwards) const;
    };

    std::ostream& operator << (std::ostream& io, Corridor const& corridor);

    void displayMedianLine(std::ostream& io, std::list<VoronoiPoint> const& skel, int xmin, int xmax, int ymin, int ymax);

    template<typename Container>
    void displayLine(std::ostream& io, Container const& container)
    {
        int line_count = 0;
        for (typename Container::const_iterator median_it = container.begin();
                median_it != container.end(); ++median_it)
        {
            if (++line_count > 5)
            {
                io << std::endl << "    ";
                line_count = 0;
            }
            io << " " << get_point(*median_it);
        }
        io << std::endl;
    }
}

#endif

