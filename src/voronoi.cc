#include "voronoi.hh"
#include "dstar.hh"
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>
#include <stdexcept>
#include <numeric>

#include <iostream>
#include <functional>

#define DEBUG
using namespace std;
using namespace boost;
using namespace nav;

list<PointSet>::iterator nav::updateConnectedSets(std::list<PointSet>& sets, PointID p)
{
    list<PointSet>::iterator it;
    for (it = sets.begin(); it != sets.end(); ++it)
    {
        if (p.isNeighbour(*it))
            break;
    }

    if (it == sets.end())
    { // not connected to any set
        sets.push_back( PointSet() );
        sets.rbegin()->insert( p );
        return --sets.end();
    }

    
    list<PointSet>::iterator added_in = it;
    PointSet& adjacent_border = *added_in;
    adjacent_border.insert(p);

    ++it;
    while (it != sets.end())
    {
        if (p.isNeighbour(*it))
        {
            adjacent_border.insert(it->begin(), it->end());
            sets.erase(it++);
        }
        else ++it;
    }

    return added_in;
}

void VoronoiPoint::offset(PointID const& v)
{
    center += v;
    for (list<PointVector>::iterator it = borders.begin(); it != borders.end(); ++it)
    {
        PointVector& border = *it;
        for (size_t i = 0; i < border.size(); ++i)
            border[i] += v;
    }
}

void VoronoiPoint::addBorderPoint(PointID const& p)
{
    bbox.update(p);

    BorderList::iterator it;
    for (it = borders.begin(); it != borders.end(); ++it)
    {
        if (p.isNeighbour(*it))
            break;
    }

    if (it == borders.end())
    {
        borders.push_back( PointVector() );
        borders.rbegin()->push_back( p );
    }
    else
    {
        PointVector& adjacent_border = *(it++);
        if (find(adjacent_border.begin(), adjacent_border.end(), p) == adjacent_border.end())
        {
            adjacent_border.push_back(p);
            while (it != borders.end())
            {
                if (p.isNeighbour(*it))
                {
                    adjacent_border.insert(adjacent_border.end(), it->begin(), it->end());
                    borders.erase(it++);
                }
                else ++it;
            }
        }
    }
}

void VoronoiPoint::mergeBorders(VoronoiPoint const& p)
{
    for (BorderList::const_iterator border = p.borders.begin(); border != p.borders.end(); ++border)
        for_each(border->begin(), border->end(), bind( mem_fn(&VoronoiPoint::addBorderPoint), this, _1));
}

VoronoiPoint::BorderList::const_iterator VoronoiPoint::findAdjacentBorder(PointID const& p) const
{
    return find_if(borders.begin(), borders.end(), boost::bind( boost::mem_fn(&PointID::isNeighbour<PointVector>), p, _1));
}

bool VoronoiPoint::isBorderAdjacent(PointID const& p) const
{
    return (findAdjacentBorder(p) != borders.end());
}


bool VoronoiPoint::isBorderAdjacent(VoronoiPoint const& p) const
{
    if (!bbox.isAdjacent(p.bbox))
        return false;

    set<vector<PointID> const*> unique_matches;

    for (BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
    {
        PointVector::const_iterator point;
        for (point = border->begin(); point != border->end(); ++point)
        {
            list< vector<PointID> >::const_iterator it = p.findAdjacentBorder(*point);
            unique_matches.insert(&(*it));
            if (it != p.borders.end())
                break;
        }
        if (point == border->end())
            return false;
    }

    // Now check for the symmetric relationship
    for (BorderList::const_iterator border = p.borders.begin(); border != p.borders.end(); ++border)
    {
        if (unique_matches.count(&(*border)))
            continue;

        PointVector::const_iterator point;
        for (point = border->begin(); point != border->end(); ++point)
        {
            if (isBorderAdjacent(*point))
                break;
        }
        if (point == border->end())
            return false;
    }
    return true;
}

bool VoronoiPoint::isSingleton() const { return borders.size() == 0; }

Point<float> VoronoiPoint::direction() const
{
    if (borders.size() == 0)
        return Point<float>();
    if (borders.size() != 2)
        throw std::runtime_error("found more than two borders in VoronoiPoint::direction()");

    BorderList::const_iterator it = borders.begin();
    PointID p1 = accumulate(it->begin(), it->end(), PointID()) / it->size();
    ++it;
    PointID p2 = accumulate(it->begin(), it->end(), PointID()) / it->size();

    return (p2 - p1).normalize();
}

bool VoronoiPoint::operator == (VoronoiPoint const& other) const
{
    return width == other.width && borders == other.borders;
}

ostream& nav::operator << (ostream& io, VoronoiPoint const& p)
{
    typedef std::list< PointVector > BorderList;
    for (BorderList::const_iterator it = p.borders.begin(); it != p.borders.end(); ++it)
    {
        io << " (";
        for (PointVector::const_iterator p = it->begin(); p != it->end(); ++p)
            io << " " << *p;
        io << ")";
    }
    return io;
}

static void displayPoint(ostream& io, list<VoronoiPoint> const& median_set, PointSet const& border_set, int x, int y)
{
    if (x < -9)
        io << "   ";
    else
        io << "  ";

    for (list<VoronoiPoint>::const_iterator it = median_set.begin(); it != median_set.end(); ++it)
        if (it->center == PointID(x, y))
        {
            io << 1;
            return;
        }

    if (border_set.count(PointID(x, y)))
        io << 0;
    else
        io << "-";
}

void nav::displayMedianLine(ostream& io, list<VoronoiPoint> const& skel, int xmin, int xmax, int ymin, int ymax)
{
    // Build the set of points that are in the border
    PointSet border_all;
    for (list<VoronoiPoint>::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        VoronoiPoint::BorderList const& borders = it->borders;
        for (VoronoiPoint::BorderList::const_iterator border = borders.begin(); border != borders.end(); ++border)
            border_all.insert(border->begin(), border->end());
    }

    io << "  Bitmap:" << endl;
    io << "     ";
    for (int x = xmin; x < xmax; ++x)
        io << " " << std::setw(2) << x;
    for (int y = ymin; y < ymax; ++y)
    {
        io << endl << "  ";
        io << std::setw(3) << y;
        for (int x = xmin; x < xmax; ++x)
            displayPoint(io, skel, border_all, x, y);
    }

    io << endl << "  Parent list:";
    for (list<VoronoiPoint>::const_iterator it = skel.begin(); it != skel.end(); ++it)
    {
        io << endl;
        io << "    " << *it << endl;
    }
    io << endl;
}

Corridor::Corridor()
    : bidirectional(false)
{
    boundary_curves[0].setGeometricResolution(1.1);
    boundary_curves[1].setGeometricResolution(1.1);
    median_curve.setGeometricResolution(1.1);
    end_types[0] = end_types[1] = 0;
}

Corridor Corridor::singleton(PointID const& p, std::string const& name)
{
    Corridor c;
    c.name = name;

    VoronoiPoint m;
    m.center = p;
    m.bbox.update(p);
    c.voronoi.push_back(m);

    c.end_regions[0].insert(p);
    c.bbox.update(p);
    c.median_bbox.update(p);
    return c;
}

template<typename _It, typename PointGetter>
_It findNearest(_It begin, _It end, PointID const& p, PointGetter get_point)
{
    float min_distance = -1;
    _It result;
    for (_It it = begin; it != end; ++it)
    {
        float d = p.distanceTo2(get_point(*it));
        if (min_distance == -1 || min_distance > d)
        {
            min_distance = d;
            result = it;
        }
    }
    return result;
}

Corridor::voronoi_iterator Corridor::findNearestMedian(PointID const& p)
{ return findNearest(voronoi.begin(), voronoi.end(), p, boost::bind(&VoronoiPoint::center, _1)); }
Corridor::voronoi_const_iterator Corridor::findNearestMedian(PointID const& p) const
{ return findNearest(voronoi.begin(), voronoi.end(), p, boost::bind(&VoronoiPoint::center, _1)); }

template<typename _It>
_It findMedianPoint(_It begin, _It end, PointID const& p)
{
    for (_It it = begin; it != end; ++it)
    {
        if (it->center == p)
            return it;
    }
    return end;
}
Corridor::voronoi_iterator Corridor::findMedianPoint(PointID const& p)
{ return ::findMedianPoint(voronoi.begin(), voronoi.end(), p); }
Corridor::voronoi_const_iterator Corridor::findMedianPoint(PointID const& p) const
{ return ::findMedianPoint(voronoi.begin(), voronoi.end(), p); }

int Corridor::findSideOf(PointID const& p) const
{
    if (end_regions[0].count(p))
	return 0;
    else if (end_regions[1].count(p))
	return 1;
    else
    {
        PointID front = voronoi.front().center;
        PointID back  = voronoi.back().center;
	float d_front = front.distance2(p);
	float d_back  = back.distance2(p);
        if (d_front > d_back)
            return 1;
        else return 0;

	cerr << name << " " << p << " is not in an end region" << endl;
	throw runtime_error("given point is not in one of the end regions");
    }
}

static int findConcatenationOrder(list<PointID> const& line0, list<PointID> const& line1)
{
    PointID front0 = line0.front();
    PointID back0  = line0.back();
    PointID front1 = line1.front();
    PointID back1  = line1.back();

    double distances[4];
    distances[0] = front0.distance2(front1);
    distances[1] = front0.distance2(back1);
    distances[2] = back0.distance2(front1);
    distances[3] = back0.distance2(back1);
    return min_element(distances, distances + 4) - distances;
}

/** Finds the right place at which to insert \c p in \c container. It returns
 * true if the point should be inserted at the end of container, and false
 * otherwise.
 *
 * Additionally, it may reorder points at the beginning/end of container so that
 * the line is properly ordered
 */
template<typename Container, typename PointAccess>
static bool findInsertSide(Container& container, PointID const&p, PointAccess get_point)
{
    if (container.size() <= 1)
	return true;

    PointID front_point = get_point(container.front());
    PointID back_point  = get_point(container.back());
    bool front_n  = front_point.isNeighbour(p);
    bool back_n   = back_point.isNeighbour(p);

    if (front_n && !back_n)
        return false;
    else if (back_n && !front_n)
        return true;
    else if (!front_n && !back_n)
    {
        if (get_point(*++container.begin()).isNeighbour(p))
            return false;
        else if (get_point(*++container.rbegin()).isNeighbour(p))
            return true;
    }

    float d_front = p.distance2(front_point);
    float d_back  = p.distance2(back_point);
    return (d_front >= d_back);
}

template<typename It, typename PointAccess>
static It lineFindLocalMinima(It it, It end, PointID const& p, PointAccess get_point)
{
    It result = it;
    float last_d = get_point(*it).distance2(p);
    for (++it; it != end; ++it)
    {
        float new_d = get_point(*it).distance2(p);
        if (new_d > last_d)
            return result;

        last_d = new_d;
        result = it;
    }
    return result;
}

template<typename Container, typename PointAccess>
static typename Container::iterator lineAppend(Container& container, PointID const& p, typename Container::value_type const& element, PointAccess get_point, bool insert_at_end)
{
    typename Container::iterator insert_place;
    if (insert_at_end)
        insert_place = container.end();
    else
        insert_place = container.begin();

    if (container.size() > 1)
    {
        if (insert_at_end)
        {
            typename Container::value_type& back = container.back();
            typename Container::value_type& one_before_back = *++container.rbegin();
            if (get_point(back).distance2(p) - 0.01 > get_point(one_before_back).distance2(p))
                --insert_place;

            //if (get_point(back).distance2(p) - 0.01 > get_point(one_before_back).distance2(p))
            //    swap(back, one_before_back);
            //else if (fabs(get_point(back).distance2(p) - get_point(one_before_back).distance2(p)) < 0.01)
            //    --insert_place;
        }
        else
        {
            typename Container::value_type& front = container.front();
            typename Container::value_type& one_after_front = *++container.begin();
            if (get_point(front).distance2(p) - 0.01 > get_point(one_after_front).distance2(p))
                ++insert_place;

            //if (get_point(front).distance2(p) - 0.01 > get_point(one_after_front).distance2(p))
            //    swap(front, one_after_front);
            //else if (fabs(get_point(front).distance2(p) - get_point(one_after_front).distance2(p)) < 0.01)
            //    ++insert_place;
        }
    }

    return container.insert(insert_place, element);
}


void Corridor::add(PointID const& p, VoronoiPoint const& descriptor, bool ordered)
{
#ifdef DEBUG
    std::cerr << "adding " << p << " " << descriptor << " to " << name << std::endl;
    cerr << endl << "voronoi:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
    cerr << endl << "boundaries[0]:";
    displayLine(cerr, boundaries[0], std::_Identity<PointID>());
    cerr << "boundaries[1]:";
    displayLine(cerr, boundaries[1], std::_Identity<PointID>());
    std::cerr << std::endl;
#endif

    if (descriptor.borders.size() != 2)
    {
        return;
        throw std::runtime_error("cannot add a point with a connectivity not 2 in a corridor");
    }

    bool insert_at_end = true;
    if (!ordered)
        insert_at_end = findInsertSide(voronoi, p, boost::bind(&VoronoiPoint::center, _1));

#ifdef DEBUG
    std::cerr << "  -- at " << (insert_at_end ? "end" : "begin") << std::endl;
#endif

    list<VoronoiPoint>::iterator insert_place =
        lineAppend(voronoi, p, descriptor, boost::bind(&VoronoiPoint::center, _1), insert_at_end);
    if (insert_place != voronoi.end())
        insert_place->center = p;
    median_bbox.update(p);
    bbox.update(p);

    extendBoundaries(descriptor, insert_at_end);
#ifdef DEBUG
    cerr << endl << "voronoi:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
    cerr << "median_bbox: " << median_bbox << endl;
    cerr << endl << "boundaries[0]:";
    displayLine(cerr, boundaries[0], std::_Identity<PointID>());
    cerr << "boundaries[1]:";
    displayLine(cerr, boundaries[1], std::_Identity<PointID>());
    cerr << "bbox: " << bbox << endl;
    std::cerr << std::endl;

#endif
}

void Corridor::add(VoronoiPoint const& p, bool ordered)
{
    add(p.center, p, ordered);
}


template<typename It>
static void deleteAlreadyInsertedPoints(list<PointID>& new_points, It it, It const end)
{
    for (; it != end; ++it)
    {
        list<PointID>::iterator new_it = find(new_points.begin(), new_points.end(), *it);
        if (new_it != new_points.end())
            new_points.erase(new_it);
    }
}

void Corridor::extendBoundaries(VoronoiPoint const& descriptor, bool insert_at_end)
{
    for (int other_idx = 0; other_idx < 2; ++other_idx)
    {
        list<PointID> other_boundary;
        if (other_idx == 0) other_boundary.insert(other_boundary.end(), descriptor.borders.front().begin(), descriptor.borders.front().end());
        else                other_boundary.insert(other_boundary.end(), descriptor.borders.back().begin(), descriptor.borders.back().end());
        if (other_boundary.empty())
            throw std::logic_error("found empty boundary in a VoronoiPoint");

        // Now remove the points that are already in the boundary to the set to
        // be inserted
        if (insert_at_end)
        {
            for (int self_idx = 0; self_idx < 2; ++self_idx)
                deleteAlreadyInsertedPoints(other_boundary, boundaries[self_idx].rbegin(), boundaries[self_idx].rend());
        }
        else
        {
            for (int self_idx = 0; self_idx < 2; ++self_idx)
                deleteAlreadyInsertedPoints(other_boundary, boundaries[self_idx].begin(), boundaries[self_idx].end());
        }

        if (other_boundary.empty())
            continue;
        PointID other_front = other_boundary.front();
        PointID other_back  = other_boundary.back();

        double distances[8] = { 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1 };
        for (int self_idx = 0; self_idx < 2; ++self_idx)
        {
            if (!boundaries[self_idx].empty())
            {
                PointID self_front = boundaries[self_idx].front();
                PointID self_back  = boundaries[self_idx].back();
                distances[self_idx * 4 + 0] = self_front.distance2(other_front);
                distances[self_idx * 4 + 1] = self_front.distance2(other_back);
                distances[self_idx * 4 + 2] = self_back.distance2(other_front);
                distances[self_idx * 4 + 3] = self_back.distance2(other_back);
            }
        }

        int min_dist = std::min_element(distances, distances + 8) - distances;
        int self_idx     = min_dist / 4;
        int self_at_end  = (min_dist % 4) / 2;
        int other_at_end = min_dist % 2;
        list<PointID>& self_boundary = boundaries[self_idx];

        if (other_at_end)
            other_boundary.reverse();
        if (insert_at_end ^ self_at_end)
            self_boundary.reverse();

#ifdef DEBUG
        cerr << "min_dist=" << min_dist << " (" << distances[min_dist] << "), appending " << other_idx << " to " << self_idx << endl;
#endif

        for (list<PointID>::const_iterator point_it = other_boundary.begin();
                point_it != other_boundary.end(); ++point_it)
        {
            PointID p = *point_it;
            lineAppend(self_boundary, p, p, std::_Identity<PointID>(), insert_at_end);
            bbox.update(p);
        }
    }

    checkConsistency();
}

bool Corridor::isBorderAdjacent(VoronoiPoint const& p) const
{
    return voronoi.front().isBorderAdjacent(p) || voronoi.back().isBorderAdjacent(p);
}

template<typename Container, typename PointGetter>
static bool checkMonotonicCurve(Container const& container, PointGetter get_point)
{
    list<PointID> queue;
    for (typename Container::const_iterator it = container.begin(); it != container.end(); ++it)
    {
        if (queue.size() < 3)
        {
            queue.push_back(get_point(*it));
            continue;
        }

        PointID direction = (queue.back() - queue.front());
        queue.push_back(get_point(*it));
        PointID new_direction = (queue.back() - queue.front());
        if (direction * new_direction < 0)
        {
            cerr << "going backwards at " << queue.front() << " => " << *(++queue.rbegin()) << " , " << queue.back() << endl;
            return false;
        }
        queue.pop_front();
    }
    return true;
}

bool Corridor::checkConsistency() const
{
#ifdef DEBUG
    bool result = true;

    // Check that the voronoi lines and the boundaries have a monotonic ordering
    if (!checkMonotonicCurve(voronoi, boost::bind(&VoronoiPoint::center, _1)))
    {
        cerr << "'" << name << "' voronoi is inconsistent: ";
        displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
        result = false;
    }
    if (!checkMonotonicCurve(boundaries[0], std::_Identity<PointID>()))
    {
        cerr << "'" << name << "' boundaries[0] is inconsistent: ";
        displayLine(cerr, boundaries[0], std::_Identity<PointID>());
        result = false;
    }
    if (!checkMonotonicCurve(boundaries[1], std::_Identity<PointID>()))
    {
        cerr << "'" << name << "' boundaries[1] is inconsistent: ";
        displayLine(cerr, boundaries[1], std::_Identity<PointID>());
        result = false;
    }

    // Verify that all connection points are in the voronoi line
    // Corridor::Connections::const_iterator conn_it;
    // for (conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
    // {
    //     PointID p = conn_it->get<0>();
    //     if (find_if(voronoi.begin(), voronoi.end(),
    //                 bind(&VoronoiPoint::center, _1) == p) == voronoi.end())
    //     {
    //         result = false;
    //         cerr << "connection point " << p << " is not in the voronoi" << endl;
    //     }
    // }

    return result;
#else
    return true;
#endif
}

template<typename It>
static It findConnectionFrom(It begin, It end, PointID const& p)
{
    for (It it = begin; it != end; ++it)
    {
        if (it->get<0>() == p)
            return it;
    }
    return end;
}

Corridor::Connections::iterator Corridor::findConnectionFrom(PointID const& p)
{ return ::findConnectionFrom(connections.begin(), connections.end(), p); }
Corridor::Connections::const_iterator Corridor::findConnectionFrom(PointID const& p) const
{ return ::findConnectionFrom(connections.begin(), connections.end(), p); }

template<typename It>
static It findConnectionTo(It begin, It end, int target_idx, PointID const& target_p)
{
    for (It it = begin; it != end; ++it)
    {
        if (it->get<1>() == target_idx && it->get<2>() == target_p)
            return it;
    }
    return end;
}

Corridor::Connections::iterator Corridor::findConnectionTo(int target_idx, PointID const& p)
{ return ::findConnectionTo(connections.begin(), connections.end(), target_idx, p); }
Corridor::Connections::const_iterator Corridor::findConnectionTo(int target_idx, PointID const& p) const
{ return ::findConnectionTo(connections.begin(), connections.end(), target_idx, p); }

template<typename T>
void reverseList(list<T>& l)
{
    if (l.size() < 2)
        return;

    typename list<T>::iterator
        end = l.end(),
        it = l.end();

    // Get one element before the last element
    --it; --it;
    while (it != l.begin())
        l.splice(l.end(), l, it--);

    l.splice(l.end(), l, l.begin());
}

void Corridor::reverse()
{
    reverseList(voronoi);
    reverseList(boundaries[0]);
    reverseList(boundaries[1]);
    swap(end_types[0], end_types[1]);
    swap(end_regions[0], end_regions[1]);
}

void Corridor::move(Corridor& other)
{
    name = other.name;
    voronoi.swap(other.voronoi);
    boundaries[0].swap(other.boundaries[0]);
    boundaries[1].swap(other.boundaries[1]);
    end_regions[0] = other.end_regions[0];
    end_regions[1] = other.end_regions[1];
    bbox = other.bbox;
    median_bbox = other.median_bbox;
}

void Corridor::buildTangent()
{
    Corridor::voronoi_const_iterator left  = voronoi.end();
    Corridor::voronoi_const_iterator right = ++voronoi.begin();
    for (Corridor::voronoi_iterator it = voronoi.begin(); it != voronoi.end(); ++it, ++left, ++right)
    {
        if (left != voronoi.end() && right != voronoi.end())
            it->tangent = Point<float>(right->center - left->center).normalize();
        else if (left != voronoi.end())
            it->tangent = Point<float>(it->center - left->center).normalize();
        else if (right != voronoi.end())
            it->tangent = Point<float>(right->center - it->center).normalize();
        else
            it->tangent = Point<float>();
    }
}

void Corridor::addConnection(PointID const& source_p, int target_idx, PointID const& target_p)
{
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->get<1>() == target_idx &&
                it->get<2>() == target_p &&
                it->get<0>() == source_p)
            return;
    }

    connections.push_back( make_tuple(source_p, target_idx, target_p));
}

bool Corridor::isConnectedTo(int other_corridor) const
{
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->get<1>() == other_corridor)
            return true;
    }
    return false;
}

void Corridor::moveConnections(size_t prev_idx, size_t new_idx)
{
    for (Connections::iterator it = connections.begin(); it != connections.end(); ++it)
    {
        int& target_idx = it->get<1>();
        if (target_idx == (int)prev_idx)
            target_idx = new_idx;
    }
}

void Corridor::removeConnectionsTo(int other_corridor)
{
    Connections::iterator it = connections.begin();
    while (it != connections.end())
    {
        if (it->get<1>() == other_corridor)
            connections.erase(it++);
        else ++it;
    }
}

template<typename Line, typename PointGetter>
static boost::tuple<int, int, double> lineMinDist(Line const& self, Line const& other, PointGetter get_point)
{
    PointID self_endpoints[2] = {
        get_point(self.front()),
        get_point(self.back())
    };
    PointID other_endpoints[2] = {
        get_point(other.front()),
        get_point(other.back())
    };

    float min_dist = -1;
    int left, right;
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            double d = self_endpoints[i].distance2(other_endpoints[j]);
            if (min_dist < 0 || min_dist > d)
            {
                left  = i;
                right = j;
                min_dist = d;
            }
        }
    }

    return make_tuple(left, right, min_dist);
}

template<typename Line>
static void mergeLines(Line& self, Line& other, int left, int right)
{
    if (left == right) // we can't splice. Reverse +self+.
    {
        self.reverse();
        left = !left;
    }

    if (left == 0)
    { // right == 1
        self.splice(self.begin(), other);
    }
    else
    { // left == 1 && right == 0
        self.splice(self.end(), other);
    }
}

template<typename Line, typename PointGetter>
static void mergeLines(Line& self, Line& other, PointGetter get_point)
{
    tuple<int, int, double> min_dist = lineMinDist(self, other, get_point);
    return mergeLines<Line>(self, other, min_dist.get<0>(), min_dist.get<1>());
}

void Corridor::merge(Corridor& other)
{
    cerr << "merging " << name << " + " << other.name << endl;

    if (bidirectional ^ other.bidirectional)
        throw runtime_error("cannot merge a bidirectional corridor with a non-bidirectional one");
    
    // It is assumed that both corridors are ordered FRONT => BACK. After the
    // merge, the ordering is therefore kept.
    {
        int self_endpoint, other_endpoint;
        tie(self_endpoint, other_endpoint, boost::tuples::ignore) =
            lineMinDist(voronoi, other.voronoi,  boost::bind(&VoronoiPoint::center, _1));

        if (self_endpoint == 0 && other_endpoint == 1)
        {
            other.merge(*this);
            move(other);
            return;
        }
    }

#ifdef DEBUG
    cerr << endl << "voronoi:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
    cerr << endl << "boundaries[0]:";
    displayLine(cerr, boundaries[0], std::_Identity<PointID>());
    cerr << "boundaries[1]:";
    displayLine(cerr, boundaries[1], std::_Identity<PointID>());

    cerr << endl << "other.voronoi:";
    displayLine(cerr, other.voronoi, boost::bind(&VoronoiPoint::center, _1));
    cerr << endl << "other.boundaries[0]:";
    displayLine(cerr, other.boundaries[0], std::_Identity<PointID>());
    cerr << "other.boundaries[1]:";
    displayLine(cerr, other.boundaries[1], std::_Identity<PointID>());
#endif

    // The boundaries are ordered in the same way than the voronoi is. The only
    // thing we need to determine is the mapping between the boundaries.
    PointID endpoint = voronoi.back().center;
    voronoi.splice(voronoi.end(), other.voronoi);

    float distances[4];
    distances[0] = boundaries[0].back().distance2(other.boundaries[0].front());
    distances[1] = boundaries[0].back().distance2(other.boundaries[1].front());
    distances[2] = boundaries[1].back().distance2(other.boundaries[0].front());
    distances[3] = boundaries[1].back().distance2(other.boundaries[1].front());
    int min_d_idx = std::min_element(distances, distances + 4) - distances;
    if (min_d_idx == 0 || min_d_idx == 3)
    {
        mergeLines(boundaries[0], other.boundaries[0], true, false);
        mergeLines(boundaries[1], other.boundaries[1], true, false);
    }
    else
    {
        mergeLines(boundaries[0], other.boundaries[1], true, false);
        mergeLines(boundaries[1], other.boundaries[0], true, false);
    }

    if (name != other.name)
        name = name + "+" + other.name;

    bbox.merge(other.bbox);
    median_bbox.merge(other.median_bbox);

    cerr << endl << "result.voronoi:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
    cerr << endl << "result.boundaries[0]:";
    displayLine(cerr, boundaries[0], std::_Identity<PointID>());
    cerr << "result.boundaries[1]:";
    displayLine(cerr, boundaries[1], std::_Identity<PointID>());

    checkConsistency();
}

bool Corridor::contains(PointID const& p) const
{
    if (!bbox.isIncluded(p))
        return false;

    return true; // buggy ...
}

bool Corridor::isNeighbour(PointID const& p) const
{
    if (!bbox.isNeighbour(p))
        return false;
    if (contains(p))
        return true;
    return true;
}

bool Corridor::isMedianNeighbour(PointID const& p) const
{
    if (!median_bbox.isNeighbour(p))
        return false;

    for (Corridor::voronoi_const_iterator it = voronoi.begin(); it != voronoi.end(); ++it)
        if (it->center.isNeighbour(p))
            return true;

    return false;
}

void Corridor::clear()
{
    median_bbox = bbox = BoundingBox();
    boundaries[0].clear();
    boundaries[1].clear();
    voronoi.clear();
    connections.clear();
}

bool Corridor::operator == (Corridor const& other) const
{
    return voronoi == other.voronoi;
}

set<int> Corridor::connectivity() const
{
    set<int> result;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
        result.insert(it->get<1>());
    return result;
}

void Corridor::buildEndRegions()
{
    end_regions[0].clear();
    end_regions[1].clear();
    PointID front = frontPoint();
    PointID back  = backPoint();

    for (Connections::const_iterator conn_it = connections.begin(); conn_it != connections.end(); ++conn_it)
    {
	float d_front = front.distance2(conn_it->get<0>());
	float d_back  = back.distance2(conn_it->get<0>());
        std::cerr << name << " is connected to idx=" << conn_it->get<1>() << ": " << conn_it->get<0>() << " <=> " << conn_it->get<1>() << std::endl;
	if (d_front <= d_back)
	    end_regions[0].insert(conn_it->get<0>());
	else
	    end_regions[1].insert(conn_it->get<0>());
    }
}

list<PointSet> Corridor::endRegions() const
{
    list<PointSet> endpoints;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
        updateConnectedSets(endpoints, it->get<0>());
    return endpoints;
}

PointID Corridor::adjacentEndpoint(PointID const& p) const
{
    for (Corridor::voronoi_const_iterator it = voronoi.begin(); it != voronoi.end(); ++it)
    {
        if (it->center.isNeighbour(p))
            return it->center;
    }
    return PointID();
}

ostream& nav::operator << (ostream& io, Corridor const& corridor)
{
//    displayMedianLine(io, corridor.voronoi, corridor.bbox.min.x - 1, corridor.bbox.max.x + 1, corridor.bbox.min.y - 1, corridor.bbox.max.y + 1);
//
//    io << "  Borders:\n";
//    io << "    " << static_cast<MedianPoint const&>(corridor);
//    io << "\n";
//    io << "  Connections:\n";
//    for (Corridor::Connections::const_iterator it = corridor.connections.begin(); it != corridor.connections.end(); ++it)
//        io << "    " << it->get<0>() << " -> [" << it->get<1>() << ": " << it->get<2>() << "]" << endl;
    return io;
}

bool Corridor::updateCurves()
{
    cerr << "generating curves for " << name << endl;
    boundary_curves[0].clear();
    boundary_curves[1].clear();
    median_curve.clear();

    boundaries[0].unique();
    boundaries[1].unique();

    if (boundaries[0].size() < 4) return false;
    for (list<PointID>::const_iterator it = boundaries[0].begin(); it != boundaries[0].end(); ++it)
        boundary_curves[0].addPoint(Eigen::Vector3d(it->x, it->y, 0));
    boundary_curves[0].update();

    if (boundaries[1].size() < 4) return false;
    for (list<PointID>::const_iterator it = boundaries[1].begin(); it != boundaries[1].end(); ++it)
        boundary_curves[1].addPoint(Eigen::Vector3d(it->x, it->y, 0));
    boundary_curves[1].update();

    Corridor::voronoi_const_iterator median_it = voronoi.begin();
    Corridor::voronoi_const_iterator const median_end = voronoi.end();
    for (; median_it != median_end; ++median_it)
        median_curve.addPoint(Eigen::Vector3d(median_it->center.x, median_it->center.y, 0));
    if (median_curve.getPointCount() < 4)
        return false;
    median_curve.update();

    boundary_curves[0].simplify(2);
    boundary_curves[1].simplify(2);
    median_curve.simplify(2);

    return true;
}

void lineOrderingDFS(list<PointID>& result, PointID cur_point, int last_direction, GridGraph& graph)
{
    static const int PROGRESSION_MASKS[8] = {
        GridGraph::RIGHT       | GridGraph::TOP_RIGHT | GridGraph::BOTTOM_RIGHT, // RIGHT
        GridGraph::RIGHT       | GridGraph::TOP_RIGHT | GridGraph::TOP,          // TOP_RIGHT
        GridGraph::TOP         | GridGraph::TOP_RIGHT | GridGraph::TOP_LEFT,     // TOP
        GridGraph::TOP         | GridGraph::LEFT      | GridGraph::TOP_LEFT,     // TOP_LEFT
        GridGraph::BOTTOM_LEFT | GridGraph::LEFT      | GridGraph::TOP_LEFT,     // LEFT
        GridGraph::BOTTOM_LEFT | GridGraph::LEFT      | GridGraph::BOTTOM,       // BOTTOM_LEFT
        GridGraph::BOTTOM_LEFT | GridGraph::BOTTOM    | GridGraph::BOTTOM_RIGHT, // BOTTOM
        GridGraph::RIGHT       | GridGraph::BOTTOM    | GridGraph::BOTTOM_RIGHT  // BOTTOM_RIGHT
    };

    // Mark this place as seen
    graph.setValue(cur_point.x, cur_point.y, 0);

    result.push_back(cur_point);
    int prefix_size = result.size();
    cerr << string(prefix_size, ' ') << "adding " << cur_point << endl;
    list<PointID> temp = result;

    GridGraph::iterator n_it        = graph.neighboursBegin(cur_point.x, cur_point.y, PROGRESSION_MASKS[last_direction]);
    for (; !n_it.isEnd(); ++n_it)
    {
        temp.resize(prefix_size);
        if (n_it.getValue() != 0) // not seen yet, propagate
        {
            lineOrderingDFS(temp, n_it.getTargetPoint(), n_it.getNeighbourIndex(), graph);
            if (result.size() < temp.size()) // found a longer line
            {
                cerr << string(prefix_size, ' ') << "updated with line starting from " << n_it.getTargetPoint() << endl;
                temp.swap(result);
            }
        }
    }
}

void Corridor::fixLineOrderings()
{
    GridGraph graph(bbox.max.x - bbox.min.x + 1, bbox.max.y - bbox.min.y + 1);

    cerr << name << " bbox: " << bbox << endl;

    // First, do the voronoi
    //
    // Note that converting the median back into the voronoi point set sucks
    // badly, and we should not even have to care about the voronoi ordering
    // anyway .. but do it for now
    cerr << name << ": fixing ordering of voronoi" << endl;
    cerr << name << " initial:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));
    list<PointID> median;
    median.resize(voronoi.size());
    transform(voronoi.begin(), voronoi.end(), 
            median.begin(), boost::bind(&VoronoiPoint::center, _1));
    fixLineOrdering(graph, median);

    list<VoronoiPoint> new_voronoi;
    while (!median.empty())
    {
        PointID p = median.back();
        median.pop_back();

        list<VoronoiPoint>::iterator it;
        for (it = voronoi.begin(); it != voronoi.end(); ++it)
        {
            if (it->center == p)
                break;
        }

        if (it == voronoi.end())
            throw std::logic_error("cannot find voronoi point in fixLineOrdering");

        new_voronoi.push_front(*it);
        voronoi.erase(it);
    }
    voronoi.swap(new_voronoi);

    cerr << name << " result:";
    displayLine(cerr, voronoi, boost::bind(&VoronoiPoint::center, _1));

    // Now do the boundary lines
    cerr << name << ": fixing ordering of boundaries[0]" << endl;
    fixLineOrdering(graph, boundaries[0]);
    cerr << name << ": fixing ordering of boundaries[1]" << endl;
    fixLineOrdering(graph, boundaries[1]);
}

void Corridor::fixLineOrdering(GridGraph& graph, list<PointID>& line)
{
    if (line.empty())
        return;

    graph.clear(0);

    // Mark all points in +line+ as not seen yet
    cerr << name << " mapped:";
    list<PointID> mapped;
    for (list<PointID>::const_iterator it = line.begin(); it != line.end(); ++it)
    {
        mapped.push_back(*it - bbox.min);
        graph.setValue(it->x - bbox.min.x, it->y - bbox.min.y, 1);
    }
    displayLine(cerr, mapped, std::_Identity<PointID>());
    cerr << endl;

    list<PointID> result;

    while (true)
    {
        PointID start_point;

        {
            list<PointID>::const_iterator it;
            for (it = line.begin(); it != line.end(); ++it)
            {
                if (graph.getValue(it->x - bbox.min.x, it->y - bbox.min.y) != 0)
                    break;
            }

            if (it == line.end())
                break; // we're done !

            // There are still some other components to fix
            start_point = *it - bbox.min;
        }
       
        cerr << "starting at " << start_point << " (" << (start_point + bbox.min) << ")" << endl;
        graph.setValue(start_point.x, start_point.y, 0);
        GridGraph::iterator n_it = graph.neighboursBegin(start_point.x, start_point.y);

        // Find the two longest lines. We always maintain line0.size() > line1.size()
        // always.
        list<PointID> line0, line1;
        list<PointID> temp;
        for (; !n_it.isEnd(); ++n_it)
        {
            if (n_it.getValue() == 0)
                continue;

            temp.clear();
            PointID root = n_it.getTargetPoint();
            lineOrderingDFS(temp, n_it.getTargetPoint(), n_it.getNeighbourIndex(), graph);
            std::cerr << "temp:" << temp.size();

            if (temp.size() > line0.size())
            {
                line0.swap(line1);
                line0.swap(temp);
            }
            else
                line1.swap(temp);

            cerr << "line0:" << line0.size() << " " << "line1:" << line1.size() << std::endl;
        }

        // Make one line out of line0 and line1. Don't forget to add bbox.min
        // back *and* add the start_point in the middle.
        line0.reverse();
        line0.push_back(start_point);
        line0.splice(line0.end(), line1);
        for (list<PointID>::iterator it = line0.begin(); it != line0.end(); ++it)
            *it = *it + bbox.min;

        // Concatenate with data already in +result+
        if (result.empty())
            result.swap(line0);
        else
        {
            int order = findConcatenationOrder(result, line0);
            if (order / 2) // attach to result.back
            {
                if (order % 2 == 1)
                {
                    // attach to line0.back
                    line0.reverse();
                    // now attach to line0.front
                }
                result.splice(result.end(), line0);
            }
            else // attach to result.front
            {
                if (order % 2 == 0)
                {
                    // attach to line0.front
                    line0.reverse();
                    // now attach to line0.back
                }
                result.splice(result.begin(), line0);
            }
        }
    }

    PointID const p_front = line.front();
    line.swap(result);
    if (p_front.distance2(line.front()) < p_front.distance2(line.back()))
        line.reverse();

}

