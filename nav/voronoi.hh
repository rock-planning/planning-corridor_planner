#ifndef NAV_VORONOI_HH
#define NAV_VORONOI_HH
#include <list>
#include <map>
#include <vector>
#include <iosfwd>
#include <boost/tuple/tuple.hpp>
#include "point.hh"

namespace Nav
{
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
    struct MedianPoint
    {
        BoundingBox bbox;
        typedef std::list< PointVector > BorderList;

        PointID center;

        /** MedianPoint maintains a list of borders, each borders being a set of
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

        MedianPoint() : width(0)  {}
        bool operator == (MedianPoint const& other) const;

        Point<float> direction() const;
        bool isSingleton() const;

        void offset(PointID const& v);

        /** Add \c p to the borders. See borders for more details */
        void addBorderPoint(PointID const& p);
        /** Merge the borders of \c p into the ones of \c this */
        void mergeBorders(MedianPoint const& p);

        /** Returns true if for each border of \c p there is at least one
         * touching border in \c this */
        bool isBorderAdjacent(MedianPoint const& p) const;
        bool isBorderAdjacent(PointID const& p) const;
    };
    std::ostream& operator << (std::ostream& io, MedianPoint const& p);
    typedef std::list<MedianPoint> MedianLine;

    class Corridor : public MedianPoint
    {
    public:
        MedianLine median;
        BoundingBox median_bbox;
        std::string name;

        typedef boost::tuple<PointID, int, PointID> ConnectionDescriptor;
        typedef std::list<ConnectionDescriptor> Connections;
        typedef Connections::iterator connection_iterator;
        Connections connections;

	PointSet end_regions[2];
	int end_types[2];
	bool bidirectional;
	void buildEndRegions();

	Corridor();

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

        /** Add the given median point to the corridor, and then change its
         * center point to the one given */
        void add(PointID const& p, MedianPoint const& median, bool ordered = false);

        /** Add a median point to the corridor, updating its border and bounding
         * box */
        void add(MedianPoint const& p, bool ordered = false);

	int findSideOf(PointID const& p) const;

        /** Merge \c corridor into this one, updating its border, median line
         * and bounding box
         */
        void merge(Corridor const& corridor);

        /** Returns the set of corridors this one is connected to */
        std::set<int> connectivity() const;

        /** Returns true if there is at least one connection from this corridor
         * to \c other_corridor
         */
        bool isConnectedTo(int other_corridor) const;

        void addConnection(PointID const& source_p, int target_idx, PointID const& target_p);

        /** Returns the iterator in \c connections whose source is \c p
         * or connections.end() if \c p is not an endpoint
         */
        Connections::iterator findConnectionFrom(PointID const& p);

        /** Returns the iterator in \c connections whose source is \c p
         * or connections.end() if \c p is not an endpoint
         */
        Connections::const_iterator findConnectionFrom(PointID const& p) const;

        /** Returns the iterator in \c connections whose target is \c p
         * or connections.end() if \c p is not an endpoint
         */
        Connections::iterator findConnectionTo(int corridor_idx, PointID const& p);

        /** Returns the iterator in \c connections whose target is \c p
         * or connections.end() if \c p is not an endpoint
         */
        Connections::const_iterator findConnectionTo(int corridor_idx, PointID const& p) const;

        /** Removes all connections that point to \c other_corridor. It does
         * not remove them on \c other_corridor
         */
        void removeConnectionsTo(int other_corridor);

        /** Returns a point of the median line which is adjacent to \c p */
        PointID adjacentEndpoint(PointID const& p) const;

        /** Returns an interator on the point of the median nearest to \c p */
        MedianLine::iterator findNearestMedian(PointID const& p);
        MedianLine::const_iterator findNearestMedian(PointID const& p) const;

        /** Returns true if the corridor invariants are met, and false otherwise
         */
        bool checkConsistency() const;

        static Corridor singleton(PointID const& p, std::string const& name = "");
    };

    std::ostream& operator << (std::ostream& io, Corridor const& corridor);

    void displayMedianLine(std::ostream& io, MedianLine const& skel, int xmin, int xmax, int ymin, int ymax);
}

#endif

