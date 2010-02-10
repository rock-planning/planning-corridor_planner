#ifndef NAV_VORONOI_HH
#define NAV_VORONOI_HH
#include <list>
#include <map>
#include <vector>
#include <iosfwd>
#include <boost/tuple/tuple.hpp>
#include "point.hh"

#include "NURBSCurve3D.hh"

namespace nav
{
    class GridGraph;
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

        Point<float> tangent;

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

        Point<float> direction() const;
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
    std::ostream& operator << (std::ostream& io, VoronoiPoint const& p);

    class Corridor
    {
    public:
        std::list<VoronoiPoint> voronoi;
        std::list<PointID> boundaries[2];

        geometry::NURBSCurve3D median_curve;
        geometry::NURBSCurve3D boundary_curves[2];

        static const int ENDPOINT_UNKNOWN = 0;
        static const int ENDPOINT_FRONT   = 1;
        static const int ENDPOINT_BACK    = 2;
        static const int ENDPOINT_BIDIR   = 3;

	int end_types[2];
	bool bidirectional;

        BoundingBox bbox;
        BoundingBox median_bbox;

        std::string name;

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
        Connections connections;

        bool isDeadEnd() const
        {
            bool has_front = false, has_back = false;
            for (Connections::const_iterator it = connections.begin();
                    it != connections.end(); ++it)
            {
                if (!has_front && !it->this_side)
                    has_front = true;
                else if (!has_back && it->this_side)
                    has_back = true;

                if (has_front && has_back) return true;
            }
            return false;
        }

	Corridor();

        void swap(Corridor& other);
        void update();

        /** Returns true if \c p is contained in this corridor */
        bool contains(PointID const& p) const;
        /** Returns true if \c p is either contained in, or touches the border of,
         * the corridor
         */
        bool isNeighbour(PointID const& p) const;

        bool isMedianNeighbour(PointID const& p) const;

        std::list<PointSet> endRegions() const;

        void clear();
        bool operator == (Corridor const& other) const;

	int findSideOf(PointID const& p) const;

        void reverse();

        void push_back(PointID const& p, VoronoiPoint const& descriptor);
        void push_back(VoronoiPoint const& descriptor);

        /** Merge \c corridor into this one, updating its border, median line
         * and bounding box
         */
        void merge(Corridor& corridor);

        /** Returns the set of corridors this one is connected to */
        std::set<int> connectivity() const;

        /** Returns true if there is at least one connection from this corridor
         * to \c other_corridor
         */
        bool isConnectedTo(int other_corridor) const;

        void addConnection(bool side, int target_idx, bool target_side);

        /** Removes all connections that point to \c other_corridor. It does
         * not remove them on \c other_corridor
         */
        void removeConnectionsTo(int other_corridor);

        /** Change the connection definition so that all connections previously
         * pointing to +prev_idx+ are now pointing to +new_idx+
         */
        void moveConnections(size_t prev_idx, size_t new_idx);

        size_t size() const { return voronoi.size(); }

        VoronoiPoint& front() { return voronoi.front(); }
        VoronoiPoint& back()  { return voronoi.back(); }
        VoronoiPoint const& front() const { return voronoi.front(); }
        VoronoiPoint const& back() const  { return voronoi.back(); }

        PointID frontPoint() const { return voronoi.front().center; }
        PointID backPoint() const { return voronoi.back().center; }
        PointID getEndpoint(bool side) const
        { return side ? backPoint() : frontPoint(); }

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

        //void fixLineOrdering(GridGraph& graph, std::list<PointID>& line);
        //void fixLineOrderings();

        static Corridor singleton(PointID const& p, std::string const& name = "");

        bool updateCurves();

        void buildBoundaries();
    };

    std::ostream& operator << (std::ostream& io, Corridor const& corridor);

    void displayMedianLine(std::ostream& io, std::list<VoronoiPoint> const& skel, int xmin, int xmax, int ymin, int ymax);

    template<typename Container, typename PointGetter>
    void displayLine(std::ostream& io, Container const& container, PointGetter get_point)
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

