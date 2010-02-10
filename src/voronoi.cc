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

template<typename Container0, typename Container1>
static int findConcatenationOrder(Container0 const& line0, Container1 const& line1)
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
    // for (BorderList::const_iterator border = p.borders.begin(); border != p.borders.end(); ++border)
    // {
    //     if (unique_matches.count(&(*border)))
    //         continue;

    //     PointVector::const_iterator point;
    //     for (point = border->begin(); point != border->end(); ++point)
    //     {
    //         if (isBorderAdjacent(*point))
    //             break;
    //     }
    //     if (point == border->end())
    //         return false;
    // }
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

    c.bbox.update(p);
    c.median_bbox.update(p);
    return c;
}

Corridor::voronoi_iterator Corridor::findNearestMedian(PointID const& p)
{ return findNearest(voronoi.begin(), voronoi.end(), p); }
Corridor::voronoi_const_iterator Corridor::findNearestMedian(PointID const& p) const
{ return findNearest(voronoi.begin(), voronoi.end(), p); }

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

bool Corridor::isBorderAdjacent(VoronoiPoint const& p) const
{
    return voronoi.front().isBorderAdjacent(p) || voronoi.back().isBorderAdjacent(p);
}

template<typename Container>
static bool checkMonotonicCurve(Container const& container)
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
    if (!checkMonotonicCurve(voronoi))
    {
        cerr << "'" << name << "' voronoi is inconsistent: ";
        displayLine(cerr, voronoi);
        result = false;
    }
    if (!checkMonotonicCurve(boundaries[0]))
    {
        cerr << "'" << name << "' boundaries[0] is inconsistent: ";
        displayLine(cerr, boundaries[0]);
        result = false;
    }
    if (!checkMonotonicCurve(boundaries[1]))
    {
        cerr << "'" << name << "' boundaries[1] is inconsistent: ";
        displayLine(cerr, boundaries[1]);
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
    std::swap(end_types[0], end_types[1]);
}

void Corridor::swap(Corridor& other)
{
    std::swap(name, other.name);
    voronoi.swap(other.voronoi);
    boundaries[0].swap(other.boundaries[0]);
    boundaries[1].swap(other.boundaries[1]);
    std::swap(bbox, other.bbox);
    std::swap(median_bbox, other.median_bbox);
    connections.swap(other.connections);
}

void Corridor::push_back(PointID const& p, VoronoiPoint const& descriptor)
{
    voronoi.push_back(descriptor);
    voronoi.back().center = p;
}
void Corridor::push_back(VoronoiPoint const& descriptor)
{ return push_back(descriptor.center, descriptor); }

void Corridor::update()
{
    // Compute the tangent along the voronoi line, add the points to the
    // boundary sets and update the bounding boxes while we're at it
    Corridor::voronoi_const_iterator left  = voronoi.end();
    Corridor::voronoi_const_iterator right = ++voronoi.begin();
    Point<float> last_slice_direction;
    int boundary0 = 0, boundary1 = 1;
    for (Corridor::voronoi_iterator it = voronoi.begin(); it != voronoi.end(); ++it, ++left, ++right)
    {
        if (it->borders.size() != 2)
            continue;

	if (it != voronoi.begin())
	{
	    Point<float> slice_direction = it->direction();
	    if (slice_direction * last_slice_direction < 0)
		std::swap(boundary0, boundary1);
	}
	copy(it->borders.front().begin(), it->borders.front().end(),
		back_inserter(boundaries[boundary0]));
	copy(it->borders.back().begin(), it->borders.back().end(),
		back_inserter(boundaries[boundary1]));
	last_slice_direction = it->direction();

        median_bbox.update(it->center);

        if (left != voronoi.end() && right != voronoi.end())
            it->tangent = Point<float>(right->center - left->center).normalize();
        else if (left != voronoi.end())
            it->tangent = Point<float>(it->center - left->center).normalize();
        else if (right != voronoi.end())
            it->tangent = Point<float>(right->center - it->center).normalize();
        else
            it->tangent = Point<float>();
    }
    bbox.merge(median_bbox);

    // Fix the boundary lines. The way we added the points is completely wrong
    // (obviously)
    //
    // We use a method akin to the one used by SkeletonExtraction to extract the
    // skeleton
    for_each(boundaries[0].begin(), boundaries[0].end(),
	    bind(&BoundingBox::update, ref(bbox), _1));
    for_each(boundaries[1].begin(), boundaries[1].end(),
	    bind(&BoundingBox::update, ref(bbox), _1));

    GridGraph temp(bbox.max.x - bbox.min.x + 1, bbox.max.y - bbox.min.y + 1);
    fixLineOrdering(temp, boundaries[0]);
    fixLineOrdering(temp, boundaries[1]);

    for (int boundary_idx = 0; boundary_idx < 2; ++boundary_idx)
    {
	double distances[4];
	distances[0] = boundaries[boundary_idx].front().distance2(frontPoint());
	distances[1] = boundaries[boundary_idx].back().distance2(backPoint());
	distances[2] = boundaries[boundary_idx].front().distance2(backPoint());
	distances[3] = boundaries[boundary_idx].back().distance2(frontPoint());
	int min_dist = min_element(distances, distances + 4) - distances;

	if (min_dist > 1)
	    boundaries[boundary_idx].reverse();
    }

    cerr << "corridor " << name << endl;
    cerr << "   bbox: " << bbox << endl;
    cerr << "   median_bbox: " << median_bbox << endl;
    cerr << "   voronoi: ";
    displayLine(cerr, voronoi);
    cerr << "   boundaries[0]";
    displayLine(cerr, boundaries[0]);
    cerr << "   boundaries[1]";
    displayLine(cerr, boundaries[1]);
}

void Corridor::addConnection(bool side, int target_idx, bool target_side)
{
    for (Connections::const_iterator it = connections.begin();
	    it != connections.end(); ++it)
    {
        if (it->target_idx == target_idx &&
                it->target_side == target_side &&
                it->this_side == side)
            return;
    }

    connections.push_back( ConnectionDescriptor(side, target_idx, target_side));
}

bool Corridor::isConnectedTo(int other_corridor) const
{
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->target_idx == other_corridor)
            return true;
    }
    return false;
}

void Corridor::moveConnections(size_t prev_idx, size_t new_idx)
{
    for (Connections::iterator it = connections.begin(); it != connections.end(); ++it)
    {
        int& target_idx = it->target_idx;
        if (target_idx == (int)prev_idx)
            target_idx = new_idx;
    }
}

void Corridor::removeConnectionsTo(int other_corridor)
{
    Connections::iterator it = connections.begin();
    while (it != connections.end())
    {
        if (it->target_idx == other_corridor)
            connections.erase(it++);
        else ++it;
    }
}

template<typename Line>
static boost::tuple<int, int, double> lineMinDist(Line const& self, Line const& other)
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

template<typename Line>
static void mergeLines(Line& self, Line& other)
{
    tuple<int, int, double> min_dist = lineMinDist(self, other);
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
            lineMinDist(voronoi, other.voronoi);

        if (self_endpoint == 0 && other_endpoint == 1)
        {
            other.merge(*this);
            swap(other);
            return;
        }
    }

#ifdef DEBUG
    cerr << endl << "voronoi:";
    displayLine(cerr, voronoi);
    cerr << endl << "boundaries[0]:";
    displayLine(cerr, boundaries[0]);
    cerr << "boundaries[1]:";
    displayLine(cerr, boundaries[1]);

    cerr << endl << "other.voronoi:";
    displayLine(cerr, other.voronoi);
    cerr << endl << "other.boundaries[0]:";
    displayLine(cerr, other.boundaries[0]);
    cerr << "other.boundaries[1]:";
    displayLine(cerr, other.boundaries[1]);
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
    displayLine(cerr, voronoi);
    cerr << endl << "result.boundaries[0]:";
    displayLine(cerr, boundaries[0]);
    cerr << "result.boundaries[1]:";
    displayLine(cerr, boundaries[1]);

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
        result.insert(it->target_idx);
    return result;
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

    if (boundaries[0].size() < 2) return false;
    for (list<PointID>::const_iterator it = boundaries[0].begin(); it != boundaries[0].end(); ++it)
        boundary_curves[0].addPoint(Eigen::Vector3d(it->x, it->y, 0));
    boundary_curves[0].update();

    if (boundaries[1].size() < 2) return false;
    for (list<PointID>::const_iterator it = boundaries[1].begin(); it != boundaries[1].end(); ++it)
        boundary_curves[1].addPoint(Eigen::Vector3d(it->x, it->y, 0));
    boundary_curves[1].update();

    Corridor::voronoi_const_iterator median_it = voronoi.begin();
    Corridor::voronoi_const_iterator const median_end = voronoi.end();
    for (; median_it != median_end; ++median_it)
        median_curve.addPoint(Eigen::Vector3d(median_it->center.x, median_it->center.y, 0));
    if (median_curve.getPointCount() < 2)
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

void Corridor::fixLineOrdering(GridGraph& graph, list<PointID>& line)
{
    if (line.empty())
        return;

    graph.clear(0);

    // Mark all points in +line+ as not seen yet
    list<PointID> mapped;
    for (list<PointID>::const_iterator it = line.begin(); it != line.end(); ++it)
    {
        mapped.push_back(*it - bbox.min);
        graph.setValue(it->x - bbox.min.x, it->y - bbox.min.y, 1);
    }

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
            if (order / 2) // attach to result.front
            {
                if (order % 2 == 0)
                {
                    // attach to line0.front
                    line0.reverse();
                    // now attach to line0.back
                }
                result.splice(result.begin(), line0);
            }
            else // attach to result.front
            {
                if (order % 2 == 1)
                {
                    // attach to line0.back
                    line0.reverse();
                    // now attach to line0.front
                }
                result.splice(result.end(), line0);
            }
        }
    }

    line.swap(result);
}

