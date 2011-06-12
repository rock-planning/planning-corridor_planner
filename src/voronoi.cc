#include "voronoi.hh"
#include <nav_graph_search/dstar.hpp>
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>
#include <stdexcept>
#include <numeric>

#include <iostream>
#include <functional>

#undef DEBUG
#ifdef DEBUG
#define DEBUG_OUT(x) cerr << x << endl;
#else
#define DEBUG_OUT(x)
#endif

using namespace std;
using namespace boost;
using namespace corridor_planner;

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

list<PointSet>::iterator corridor_planner::updateConnectedSets(std::list<PointSet>& sets, PointID p)
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

nav_graph_search::Point<float> VoronoiPoint::direction() const
{
    if (borders.size() == 0)
        return nav_graph_search::Point<float>();
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

ostream& corridor_planner::operator << (ostream& io, VoronoiPoint const& p)
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

void corridor_planner::displayMedianLine(ostream& io, list<VoronoiPoint> const& skel, int xmin, int xmax, int ymin, int ymax)
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
    : width_curve(0.5, 2), bidirectional(false)
{
    boundary_curves[0].setGeometricResolution(0.5);
    boundary_curves[1].setGeometricResolution(0.5);
    median_curve.setGeometricResolution(0.5);
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

bool Corridor::findSideOf(PointID const& p) const
{
    PointID front = voronoi.front().center;
    PointID back  = voronoi.back().center;
    float d_front = front.distance2(p);
    float d_back  = back.distance2(p);
    if (d_front > d_back)
	return 1;
    else return 0;

    DEBUG_OUT(name << " " << p << " is not in an end region");
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
            DEBUG_OUT("going backwards at " << queue.front() << " => " << *(++queue.rbegin()) << " , " << queue.back());
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

    // Verify that we don't have duplicate connections

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

    typename list<T>::iterator it = l.end();

    // Get one element before the last element
    --it; --it;
    while (it != l.begin())
        l.splice(l.end(), l, it--);

    l.splice(l.end(), l, l.begin());
}

void Corridor::reverse()
{
    name = "rev(" + name + ")";
    reverseList(voronoi);
    reverseList(boundaries[0]);
    reverseList(boundaries[1]);
    boundaries[0].swap(boundaries[1]);
    std::swap(end_types[0], end_types[1]);
    dcost = -dcost;
}

void Corridor::swap(Corridor& other)
{
    std::swap(name, other.name);
    std::swap(dcost, other.dcost);
    std::swap(bidirectional, other.bidirectional);
    voronoi.swap(other.voronoi);
    boundaries[0].swap(other.boundaries[0]);
    boundaries[1].swap(other.boundaries[1]);
    std::swap(bbox, other.bbox);
    std::swap(median_bbox, other.median_bbox);
    std::swap(min_width, other.min_width);
    std::swap(max_width, other.max_width);
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
    boundaries[0].clear();
    boundaries[1].clear();

    // Compute the tangent along the voronoi line, add the points to the
    // boundary sets and update the bounding boxes while we're at it
    Corridor::voronoi_const_iterator left  = voronoi.end();
    Corridor::voronoi_const_iterator right = ++voronoi.begin();
    nav_graph_search::Point<float> last_slice_direction;
    int boundary0 = 0, boundary1 = 1;
    min_width = voronoi.front().width, max_width = min_width;
    for (Corridor::voronoi_iterator it = voronoi.begin(); it != voronoi.end(); ++it, ++left, ++right)
    {
        if (it->borders.size() != 2)
            continue;

        float this_width = it->width;
        if (this_width < min_width)
            min_width = this_width;
        if (this_width > max_width)
            max_width = this_width;

	if (it != voronoi.begin())
	{
            nav_graph_search::Point<float> slice_direction = it->direction();
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
            it->tangent = nav_graph_search::Point<float>(right->center - left->center).normalize();
        else if (left != voronoi.end())
            it->tangent = nav_graph_search::Point<float>(it->center - left->center).normalize();
        else if (right != voronoi.end())
            it->tangent = nav_graph_search::Point<float>(right->center - it->center).normalize();
        else
            it->tangent = nav_graph_search::Point<float>();
    }
    bbox.merge(median_bbox);
    for_each(boundaries[0].begin(), boundaries[0].end(),
	    bind(&BoundingBox::update, ref(bbox), _1));
    for_each(boundaries[1].begin(), boundaries[1].end(),
	    bind(&BoundingBox::update, ref(bbox), _1));

    GridGraph temp(bbox.max.x - bbox.min.x + 1, bbox.max.y - bbox.min.y + 1);

    // Fix the boundary lines. The way we added the points is completely wrong
    // (obviously)
    //
    // We use a method akin to the one used by SkeletonExtraction to extract the
    // skeleton
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

    DEBUG_OUT("corridor " << name);
    DEBUG_OUT("   bbox: " << bbox);
    DEBUG_OUT("   median_bbox: " << median_bbox);
#ifdef DEBUG
    cerr << "   voronoi: ";
    displayLine(cerr, voronoi);
    cerr << "   boundaries[0]";
    displayLine(cerr, boundaries[0]);
    cerr << "   boundaries[1]";
    displayLine(cerr, boundaries[1]);
#endif
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
    Connections::iterator it = connections.begin();
    while (it != connections.end())
    {
        if (it->target_idx == (int)prev_idx)
        {
            ConnectionDescriptor conn = *it;
            connections.erase(it++);
            addConnection(conn.this_side, new_idx, conn.target_side);
        }
        else ++it;
    }
}

void Corridor::removeConnection(int target_idx, bool target_side)
{
    for (Connections::iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->target_idx == target_idx && it->target_side == target_side)
        {
            connections.erase(it++);
            return;
        }
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

void Corridor::concat(Corridor& other)
{
#ifdef DEBUG
    DEBUG_OUT("merging " << name << " + " << other.name);
#endif

    if (bidirectional ^ other.bidirectional)
        throw runtime_error("cannot merge a bidirectional corridor with a non-bidirectional one");

#ifdef DEBUG
    cerr << endl << name << ".concat(" << other.name << ")";
    cerr << endl << name << ".voronoi:";
    displayLine(cerr, voronoi);
    cerr << endl << name << ".boundaries[0]:";
    displayLine(cerr, boundaries[0]);
    cerr << name << ".boundaries[1]:";
    displayLine(cerr, boundaries[1]);

    cerr << endl << other.name << ".voronoi:";
    displayLine(cerr, other.voronoi);
    cerr << endl << other.name << ".boundaries[0]:";
    displayLine(cerr, other.boundaries[0]);
    cerr << other.name << ".boundaries[1]:";
    displayLine(cerr, other.boundaries[1]);

    checkConsistency();
#endif
    
    voronoi.splice(voronoi.end(), other.voronoi);
    double distances[4];
    distances[0] = boundaries[0].back().distance2(other.boundaries[0].front());
    distances[1] = boundaries[0].back().distance2(other.boundaries[1].front());
    distances[2] = boundaries[1].back().distance2(other.boundaries[0].front());
    distances[3] = boundaries[1].back().distance2(other.boundaries[1].front());
    int min_distance = min_element(distances, distances + 4) - distances;

    if (this->min_width > other.min_width)
        this->min_width = other.min_width;
    if (this->max_width < other.max_width)
        this->max_width = other.max_width;

    if (min_distance == 0 || min_distance == 3)
    {
        // Remove from other.boundaries[x] the points that are already in
        // boundaries[x]
        for (int boundary_idx = 0; boundary_idx < 2; ++boundary_idx)
        {
            while (true)
            {
                PointID p = other.boundaries[boundary_idx].front();
                if (find(boundaries[boundary_idx].rbegin(), boundaries[boundary_idx].rend(), p) != boundaries[boundary_idx].rend())
                    other.boundaries[boundary_idx].pop_front();
                else
                    break;
            }
        }
        boundaries[0].splice(boundaries[0].end(), other.boundaries[0]);
        boundaries[1].splice(boundaries[1].end(), other.boundaries[1]);
    }
    else
    {
        // Remove from other.boundaries[x] the points that are already in
        // boundaries[x]
        for (int boundary_idx = 0; boundary_idx < 2; ++boundary_idx)
        {
            while (true)
            {
                PointID p = other.boundaries[!boundary_idx].front();
                if (find(boundaries[boundary_idx].rbegin(), boundaries[boundary_idx].rend(), p) != boundaries[boundary_idx].rend())
                    other.boundaries[!boundary_idx].pop_front();
                else
                    break;
            }
        }
        boundaries[1].splice(boundaries[1].end(), other.boundaries[0]);
        boundaries[0].splice(boundaries[0].end(), other.boundaries[1]);
    }

    if (name != other.name)
        name = name + "+" + other.name;

    bbox.merge(other.bbox);
    median_bbox.merge(other.median_bbox);

#ifdef DEBUG
    cerr << endl << "result.voronoi:";
    displayLine(cerr, voronoi);
    cerr << endl << "result.boundaries[0]:";
    displayLine(cerr, boundaries[0]);
    cerr << "result.boundaries[1]:";
    displayLine(cerr, boundaries[1]);

    checkConsistency();
#endif
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

std::set<int> Corridor::connectivity(bool side) const
{
    set<int> result;
    for (Connections::const_iterator it = connections.begin(); it != connections.end(); ++it)
    {
        if (it->this_side == side)
            result.insert(it->target_idx);
    }
    return result;
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

ostream& corridor_planner::operator << (ostream& io, Corridor const& corridor)
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

void Corridor::updateCurves(double discount_factor)
{
    boundary_curves[0].clear();
    boundary_curves[1].clear();
    median_curve.clear();

    double const new_point_factor = 1.0 - discount_factor;
    double const simplification_tolerance = 0.5;

    Corridor::voronoi_const_iterator median_it = voronoi.begin();
    Corridor::voronoi_const_iterator const median_end = voronoi.end();
    PointID first_point = voronoi.front().center;
    base::Vector3d last_point = base::Vector3d(first_point.x, first_point.y, 0);
    vector<base::Vector3d> median_points;
    for (; median_it != median_end; ++median_it)
    {
        PointID p  = median_it->center;
        base::Vector3d point = new_point_factor * base::Vector3d(p.x, p.y, 0) + discount_factor * last_point;
        if (median_points.empty() || point != median_points.back())
            median_points.push_back(point);
        last_point = point;
    }
    median_curve.interpolate(median_points);
    median_curve.simplify(simplification_tolerance);

    for (int boundary_idx = 0; boundary_idx < 2; ++boundary_idx)
    {
        vector<base::Vector3d> boundary_points;
        if (boundaries[boundary_idx].empty())
        {
            // We are dealing with one of the endpoint corridors
            // Use the median curve position
            boundary_points.push_back(last_point);
        }
        else
        {
            base::Vector3d last_point = base::Vector3d(boundaries[boundary_idx].front().x, boundaries[boundary_idx].front().y, 0);
            for (list<PointID>::const_iterator it = boundaries[boundary_idx].begin(); it != boundaries[boundary_idx].end(); ++it)
            {
                base::Vector3d point = new_point_factor * base::Vector3d(it->x, it->y, 0) + discount_factor * last_point;
                if (boundary_points.empty() || boundary_points.back() != point)
                    boundary_points.push_back(point);
                last_point = point;
            }
        }

        boundary_curves[boundary_idx].interpolate(boundary_points);
        boundary_curves[boundary_idx].simplify(simplification_tolerance);
    }

    // Check if we must swap boundaries[0] and boundaries[1]
    //
    // First get a general corridor direction
    base::Vector3d median_p, median_t;
    base::Vector3d boundaries_p[2], boundaries_t[2];
    tie(median_p, median_t) = median_curve.getPointAndTangent(median_curve.getStartParam());
    for (int i = 0; i < 2; ++i)
        tie(boundaries_p[i], boundaries_t[i]) = boundary_curves[i].getPointAndTangent(boundary_curves[i].getStartParam());

    // NOTE: singletons return a zero tangent, so we can do that (we are only
    // interested in the cross product sign later on). I.e. we don't need to
    // take the mean
    base::Vector3d corridor_dir = median_t;
    if (median_curve.isSingleton())
    {
        if (boundary_curves[0].isSingleton())
            corridor_dir = boundaries_t[1];
        else
            corridor_dir = boundaries_t[0];
    }

    if ((boundaries_p[0] - median_p).cross(corridor_dir).z() < 0)
        std::swap(boundary_curves[0], boundary_curves[1]);

    // Finally, "cut" the median curve at corridor boundaries. Otherwise,
    // joining corridors to make a path is going to be tricky, as all median
    // curves would go to the center of the crossroads.
    if (!median_curve.isSingleton())
    {
        double cut_start_t = median_curve.getStartParam(), cut_end_t = median_curve.getEndParam();
        bool has_cut_start = false, has_cut_end = false;

        base::Vector3d boundary_startp[2], boundary_endp[2];
        for (int i = 0; i < 2; ++i)
        {
            boundary_startp[i] = boundary_curves[i].getPoint(boundary_curves[i].getStartParam());
            boundary_endp[i]   = boundary_curves[i].getPoint(boundary_curves[i].getEndParam());
        }

        std::vector<double> points;
        std::vector< std::pair<double, double> > curves;
        {
            base::Vector3d normal = base::Vector3d::UnitZ().cross(boundary_startp[1] - boundary_startp[0]);
            median_curve.findLineIntersections(boundary_startp[0], normal, points, curves, 0.01);
            double min_t = median_curve.getEndParam();
            has_cut_start = false;
            if (points.empty() && curves.empty())
                min_t = median_curve.getStartParam();
            else
            {
                if (!points.empty())
                    min_t = std::min(min_t, *min_element(points.begin(), points.end()));
                if (!curves.empty())
                    min_t = std::min(min_t, min_element(curves.begin(), curves.end())->first);

                base::Vector3d p = median_curve.getPoint(min_t);
                if ((boundary_startp[0] - p).dot(boundary_startp[1] - p) < 0)
                {
                    has_cut_start = true;
                    cut_start_t = min_t;
                }
            }
        }

        points.clear(); curves.clear();
        {
            base::Vector3d normal = base::Vector3d::UnitZ().cross(boundary_endp[1] - boundary_endp[0]);
            median_curve.findLineIntersections(boundary_endp[0], normal, points, curves, 0.01);
            double max_t = median_curve.getStartParam();

            has_cut_end = false;
            if (points.empty() && curves.empty())
                max_t = median_curve.getEndParam();
            else
            {
                if (!points.empty())
                    max_t = std::max(max_t, *max_element(points.begin(), points.end()));
                if (!curves.empty())
                    max_t = std::max(max_t, max_element(curves.begin(), curves.end())->second);

                base::Vector3d p = median_curve.getPoint(max_t);
                if ((boundary_endp[0] - p).dot(boundary_endp[1] - p) < 0)
                {
                    has_cut_end = true;
                    cut_end_t = max_t;
                }
            }
        }

        if (!has_cut_start && !has_cut_end)
        {
            // The median curve might be OUTSIDE the corridor (it is possible
            // for very small corridors in complicated areas). Reduce it to a
            // line between the centers of the in and out segments
            //
            // To discriminate, we look whether the end point is closer than the
            // start point to the start segment
            Eigen::Vector3d median_p[2] = { median_curve.getStartPoint(), median_curve.getEndPoint() };
            Eigen::Vector3d segment((boundary_startp[1] - boundary_startp[0]).normalized());
            Eigen::Vector3d normal(-segment.y(), segment.x(), 0);
            double d[2];
            for (int i = 0; i < 2; ++i)
                d[i] = fabs((median_p[i] - boundary_startp[0]).dot(segment));

            if (d[0] > d[1])
            {
                vector<base::Vector3d> line;
                line.push_back((boundary_startp[0] + boundary_startp[1]) / 2);
                line.push_back((boundary_endp[0] + boundary_endp[1]) / 2);
                if (line.front() == line.back())
                    median_curve.setSingleton(line.front());
                else
                    median_curve.interpolate(line);
            }
        }
        else
            median_curve.crop(cut_start_t, cut_end_t);
    }
    updateWidthCurve();
}

void Corridor::updateWidthCurve()
{
    width_curve.clear();

    typedef base::geometry::Spline<1>::vector_t point_t;
    std::vector<point_t> width_points;
    std::vector<double> parameters;
    double delta = (median_curve.getEndParam() - median_curve.getStartParam()) / voronoi.size();
    for (double t = median_curve.getStartParam();
            t < median_curve.getEndParam(); t += delta)
    {
        base::Vector3d p = median_curve.getPoint(t);
        corridor_planner::Corridor::voronoi_const_iterator median_it =
            findNearestMedian(corridor_planner::PointID(p.x(), p.y()));

        float this_width = median_it->width;
        parameters.push_back(t);

        point_t width_p;
        width_p(0, 0) = this_width;
        width_points.push_back(width_p);
    }

    {
        point_t width_p;
        width_p(0, 0) = voronoi.back().width ;
        width_points.push_back(width_p);
        parameters.push_back( median_curve.getEndParam() );
    }

    width_curve.interpolate(width_points, parameters);
    width_curve.simplify(0.5);
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
    //DEBUG_OUT(string(prefix_size, ' ') << "adding " << cur_point);
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
                //DEBUG_OUT(string(prefix_size, ' ') << "updated with line starting from " << n_it.getTargetPoint());
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
       
        //DEBUG_OUT("starting at " << start_point << " (" << (start_point + bbox.min) << ")");
        graph.setValue(start_point.x, start_point.y, 0);
        GridGraph::iterator n_it = graph.neighboursBegin(start_point.x, start_point.y);

        // Find the two longest lines. We always maintain line0.size() > line1.size()
        list<PointID> line0, line1;
        list<PointID> temp;
        for (; !n_it.isEnd(); ++n_it)
        {
            if (n_it.getValue() == 0)
                continue;

            temp.clear();
            lineOrderingDFS(temp, n_it.getTargetPoint(), n_it.getNeighbourIndex(), graph);
            //std::cerr << "temp:" << temp.size();

            if (temp.size() > line0.size())
            {
                line0.swap(line1);
                line0.swap(temp);
            }
            else
                line1.swap(temp);

            //cerr << "line0:" << line0.size() << " " << "line1:" << line1.size() << std::endl;
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
            DEBUG_OUT("looking to concatenate new=(" << line0.front() << "," << line0.back() << ") to current=(" << result.front() << "," << result.back() << ")");
            DEBUG_OUT("concatenation order: " << order);
            if (order / 2 == 0) // attach to result.front
            {
                if (order % 2 == 0)
                {
                    // attach to line0.front
                    line0.reverse();
                    // now attach to line0.back
                }
                DEBUG_OUT("attaching (" << line0.front() << "," << line0.back() << ") before (" << result.front() << "," << result.back() << ")");
                result.splice(result.begin(), line0);
            }
            else // attach to result.back
            {
                if (order % 2 == 1)
                {
                    // attach to line0.back
                    line0.reverse();
                    // now attach to line0.front
                }
                DEBUG_OUT("attaching (" << line0.front() << "," << line0.back() << ") after (" << result.front() << "," << result.back() << ")");
                result.splice(result.end(), line0);
            }
        }
    }

    line.swap(result);
}

void Corridor::moveOutgoingConnections(Corridor& other_corridor)
{
    connections.splice(connections.end(), other_corridor.connections);
}

float Corridor::getCostDelta(bool in_side) const
{
    if (in_side)
        return -dcost;
    else return dcost;
}

