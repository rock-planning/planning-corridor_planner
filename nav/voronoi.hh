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
        typedef std::list< PointSet > BorderList;

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
    typedef std::map<PointID, MedianPoint> MedianLine;

    struct Corridor : public MedianPoint
    {
        MedianLine median;
        BoundingBox median_bbox;

        typedef std::list< boost::tuple<PointID, int, PointID> > Connections;
        typedef Connections::iterator connection_iterator;
        Connections connections;

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

        /** Add a median point to the corridor, updating its border and bouding
         * box */
        void add(PointID const& p, MedianPoint const& descriptor);
        /** Add a median point to the corridor, updating its border and bouding
         * box */
        void add(std::pair<PointID, MedianPoint> const& p);
        /** Merge \c corridor into this one, updating its border, median line
         * and bounding box
         */
        void merge(Corridor const& corridor);

        /** Returns true if there is at least one connection from this corridor
         * to \c other_corridor
         */
        bool isConnectedTo(int other_corridor) const;

        /** Removes all connections that point to \c other_corridor. It does
         * not remove them on \c other_corridor
         */
        void removeConnectionsTo(int other_corridor);

        /** Returns a point of the median line which is adjacent to \c p */
        PointID adjacentEndpoint(PointID const& p) const;
    };
    std::ostream& operator << (std::ostream& io, Corridor const& corridor);

    void displayMedianLine(std::ostream& io, MedianLine const& skel, int xmin, int xmax, int ymin, int ymax);
}

#endif

