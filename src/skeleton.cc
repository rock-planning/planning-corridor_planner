#include "skeleton.hh"
#include <limits>
#include <boost/bind.hpp>
#include <algorithm>
#include "pool_allocator.hh"
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <iomanip>
#include <map>

using namespace boost;
using namespace std;
using namespace nav;

SkeletonExtraction::SkeletonExtraction(size_t width, size_t height)
    : width(width)
    , height(height)
    , heightmap(width * height)
{
    fill(heightmap.begin(), heightmap.end(), 0);
}

SkeletonExtraction::~SkeletonExtraction()
{
}

PointID SkeletonExtraction::pointFromPtr(height_t const* ptr) const
{
    uint32_t offset = ptr - &heightmap[0];
    return PointID(offset % width, offset / width);
}

list<VoronoiPoint> SkeletonExtraction::process()
{
    const int32_t addVal[8] = { 1, 2, 2, 1, 2, 2, 1, 1 };
    const int32_t displacement[8] = {
        -width, -width - 1, -width + 1,
        width, width - 1, width + 1, -1, +1 };

    // During propagation, we need to maintain only three sets:
    //  * the set of lowest distance d (candidates)
    //  * the set of d + 1 (direct propagation) (borders[0])
    //  * the set of d + 2 (diagonal propagation) (borders[1])
    //
    // There is no other distance to be considered. We therefore do not need to
    // maintain a priority queue or whatever, just those three sets. Still, a
    // point can be promoted from d + 2 to d + 1, so we need to keep them as
    // std::set.
    typedef std::set<height_t*> CandidateSet;
    CandidateSet candidates;
    CandidateSet borders[2];

    // Initialize +candidates+ and +parents+ with the border points
    parents.clear();
    for (PointSet::const_iterator it = border.begin(); it != border.end(); ++it)
    {
        if (it->x > 0 && it->x < width - 1 && it->y > 0 && it->y < height - 1)
        {
            height_t* c_ptr = &heightmap[it->y * width + it->x];
            *c_ptr = 1;
            candidates.insert( c_ptr );
            parents[c_ptr].addBorderPoint( pointFromPtr(c_ptr) );
        }
    }

// displayHeightMap(cerr);

    // The points that are part of the median line. The parents are stored in
    // the +parents+ map.
    CandidateSet skeleton;

    // Outer loop. This will leave if all three +candidates+, +borders[0]+ and
    // +borders[1]+ are empty.
    while (!candidates.empty())
    {
        // Inner loop. Process all points in +candidates+
        while (!candidates.empty())
        {
            height_t* c_ptr   = *candidates.begin();
            height_t  c_value = *c_ptr;
            // cerr << "taking " << pointFromPtr(c_ptr) << "(" << (void*)c_ptr << ")=" << (int)c_value << endl;
            // cerr << "  with borders " << parents[c_ptr] << endl;

            candidates.erase(candidates.begin());

            // Look at the neighbours. Three situations:
            //  * the propagated cost (current_cost + edge_cost) is lower. The
            //    target node is nearer to the parent of the current node.
            //    Propagate.
            //  * the cost of the target node is the same than the propagated
            //    cost. The parents of the current node are at the same distance
            //    of the neighbour than its current parents. Merge.
            //  * the cost of the neighbour is the same than the current cost.
            //    The actual Voronoi point is in-between the current node and
            //    the neighbour. We don't have half-coordinates, so merge also.
            //    Post-processing will remove points that have the same parents.
            for (size_t i = 0; i < 8; ++i)
            {
                height_t* neighbour        = c_ptr   + displacement[i];
                height_t propagated_value = c_value + addVal[i]; 
                //cerr <<  "  " << pointFromPtr(neighbour) << "(" << (void*)neighbour << ")=" << (int)*neighbour << " ~ " << propagated_value << endl;
                if (*neighbour > propagated_value) // needs to be propagated further
                {
                    *neighbour = propagated_value;
                    if (addVal[i] == 1)
                    {
                        borders[1].erase(neighbour);
                        borders[0].insert(neighbour);
                    }
                    else
                        borders[1].insert(neighbour);

                    parents[neighbour] = parents[c_ptr];
                }
                else if (*neighbour == propagated_value || (addVal[i] == 1 && *neighbour == c_value))
                {
                    //cerr <<  "   ... found border ?" << parents[neighbour] << flush;
                    if (!parents[neighbour].isBorderAdjacent( parents[c_ptr] ))
                    {
                        //cerr << " yes" << endl;

                        skeleton.insert( neighbour );
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else if (*neighbour == propagated_value)
                    {
                        //cerr << " no" << endl;
                        parents[neighbour].mergeBorders( parents[c_ptr] );
                    }
                    else
                    {
                        //cerr << " no" << endl;
                    }
                }
            }
        }

        if (borders[0].empty())
        {
            candidates.swap(borders[1]);
            borders[0].clear();
        }
        else
        {
            candidates.swap(borders[0]);
            borders[0].swap(borders[1]);
        }
    }

    //displayHeightMap(cerr);

    list<VoronoiPoint> result;
    for (CandidateSet::const_iterator it = skeleton.begin(); it != skeleton.end(); ++it)
    {
        PointID p = pointFromPtr(*it);
        result.push_back(parents[*it]);
        VoronoiPoint& info = result.back();
        info.center = p;
        info.width = **it;
    }

    return result;
}

void SkeletonExtraction::displayHeightMap(std::ostream& io) const
{
    io << "   ";
    for (int x = 0; x < width; ++x)
        io << " " << std::setw(3) << x;
    io << endl;
    for (int y = 0; y < height; ++y)
    {
        io << std::setw(3) << y;
        for (int x = 0; x < width; ++x)
        {
            int val = (int)heightmap[y * width + x];
            if (val)
                io << " " << std::setw(3) << val;
            else
                io << "   -";
        }
        io << endl;
    }
}

bool SkeletonExtraction::isInside(int x, int y) const
{
    return heightmap[y * width + x] > 1;
}

pair<PointSet, PointSet> SkeletonExtraction::getBorderAndInside() const
{
    PointSet inside;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
        {
            if (isInside(x, y))
                inside.insert(PointID(x, y));
        }

    return make_pair(border, inside);
}

void SkeletonExtraction::initializeFromDStar(DStar const& dstar, int x, int y, float expand)
{
    typedef multimap<float, PointID, std::less<float> > OpenSet;
    OpenSet open_from_cost;
    map<PointID, OpenSet::iterator, std::less<PointID> > is_in_open;

    border.clear();
    parents.clear();
    fill(heightmap.begin(), heightmap.end(), 0);

    GridGraph const& nav_function = dstar.graph();

    float cost_max = nav_function.getValue(x, y) * expand;
    {
        PointID p0(x, y);
        OpenSet::iterator n_it = open_from_cost.insert( make_pair(0, p0) );
        is_in_open.insert( make_pair(p0, n_it) );
        heightmap[y * width + x] = MAX_DIST;
    }

    while(!open_from_cost.empty())
    {
        PointID point; float cost;
        tie(cost, point) = *open_from_cost.begin();
        open_from_cost.erase(open_from_cost.begin());
        is_in_open.erase(point);

        for (NeighbourConstIterator n = nav_function.neighboursBegin(point.x, point.y); !n.isEnd(); ++n)
        {
            float n_cost = cost + dstar.costOf(n);
            PointID n_p = PointID(n.x(), n.y());

            if (n_cost + n.getValue() <= cost_max)
            {
                if (isInside(n_p.x, n_p.y))
                {
                    map<PointID, OpenSet::iterator>::iterator open_it = is_in_open.find(n_p);
                    OpenSet::iterator n_it = open_it->second;
                    if (open_it != is_in_open.end() && n_it->first > n_cost)
                    {
                        open_from_cost.erase(open_it->second);
                        n_it = open_from_cost.insert( make_pair(n_cost, n_p) );
                        open_it->second = n_it;
                    }
                }
                else
                {
                    border.erase(n_p);
                    heightmap[n_p.y * width + n_p.x] = MAX_DIST;
                    OpenSet::iterator n_it = open_from_cost.insert( make_pair(n_cost, n_p) );
                    is_in_open.insert( make_pair(n_p, n_it) );
                }
            }
            else if (!isInside(n_p.x, n_p.y))
            {
                border.insert(n_p);
            }
        }
    }
}
list<VoronoiPoint> SkeletonExtraction::processDStar(DStar const& dstar, int x, int y, float expand)
{
    initializeFromDStar(dstar, x, y, expand);
    return process();
}

void SkeletonExtraction::initializeFromPointSets(PointSet const& inside, PointSet const& border)
{
    this->border = border;
    parents.clear();
    fill(heightmap.begin(), heightmap.end(), 0);
    for (PointSet::const_iterator it = inside.begin(); it != inside.end(); ++it)
        heightmap[it->x + it->y*width] = MAX_DIST;
}
list<VoronoiPoint> SkeletonExtraction::processPointSets(PointSet const& inside, PointSet const& border)
{
    initializeFromPointSets(inside, border);
    return process();
}

void SkeletonExtraction::registerConnections(PointID source_point, int source_idx, map<PointID, int> const& targets, vector<Corridor>& corridors)
{
    Corridor& source = corridors[source_idx];
    for (map<PointID, int>::const_iterator target_it = targets.begin(); target_it != targets.end(); ++target_it)
    {
        PointID target_point = target_it->first;
        int target_idx = target_it->second;
        Corridor& target = corridors[target_idx];

        std::cerr << "connecting " << source.name << " <=> " << target.name << std::endl;

        source.addConnection(source_point, target_idx, target_point);
        target.addConnection(target_point, source_idx, source_point);
    }
}

typedef map<PointID, list<VoronoiPoint>::const_iterator> VoronoiMap;
typedef multimap< PointID, list<VoronoiPoint> > BranchMap;

struct CorridorExtractionState
{
    GridGraph  graph;

    BranchMap  branches;
    VoronoiMap voronoiMap;

    int depth;

    CorridorExtractionState(int width, int height)
        : graph(width, height)
        , depth(0) {}

    VoronoiPoint const& front()
    {
        VoronoiMap::iterator it = voronoiMap.begin();
        return *(it->second);
    }

    void addBranch(PointID const& p, list<VoronoiPoint>& line)
    {
        cerr << string(depth, ' ') << "adding branch at " << p << " of size " << line.size() << endl;
        displayLine(cerr, line, boost::bind(&VoronoiPoint::center, _1));

        BranchMap::iterator it = branches.insert( make_pair(p, list<VoronoiPoint>()) );
        it->second.swap(line);
    }
};

bool lineOrderingDFS(PointID const& cur_point, int neighbour_mask,
        list<VoronoiPoint>& parent_line,
        CorridorExtractionState& state)
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

    bool detached  = true;
    bool branching = false;

    state.graph.setValue(cur_point.x, cur_point.y, 1);
    VoronoiMap::iterator voronoi_it = state.voronoiMap.find(cur_point);
    cerr << string(state.depth, ' ') << "visiting " << cur_point << endl;
    parent_line.push_back( *(voronoi_it->second) );
    state.voronoiMap.erase(voronoi_it);
    state.depth++;

    GridGraph::iterator n_it = state.graph.neighboursBegin(cur_point.x, cur_point.y, neighbour_mask);
    list<VoronoiPoint> result, cur_line;
    for (; !n_it.isEnd(); ++n_it)
    {
        if (n_it.getValue() < 1.5) // either not actual point, or already visited
        {
            if (n_it.getValue() > 0.5)
                detached = false;
            continue;
        }

        PointID root = n_it.getTargetPoint();
        bool detached = lineOrderingDFS(n_it.getTargetPoint(), PROGRESSION_MASKS[n_it.getNeighbourIndex()],
                result, state);

        if (cur_line.empty())
            cur_line.swap(result);
        else if (!detached)
        {
            if (cur_line.size() < result.size())
                cur_line.swap(result);
        }
        else
        {
            branching = true;
            cerr << "A " << cur_line.size() << endl;
            state.addBranch(cur_point, cur_line);
            cur_line.swap(result);
        }
    }
    
    state.depth--;
    if (!branching)
        parent_line.splice(parent_line.end(), cur_line);
    else if (!cur_line.empty())
    {
        cerr << "B " << cur_line.size() << endl;
        state.addBranch(cur_point, cur_line);
    }

    return branching || detached;
}


void SkeletonExtraction::extractBranches(list<VoronoiPoint> const& points, multimap<PointID, list<VoronoiPoint> >& branches)
{
    // Initialize a GridGraph of the right size, and mark the voronoi points in
    // it. We will use it to traverse the terrain
    //
    // At the same time, keep a mapping from PointID to voronoi points. The
    // points will be removed from there as they are traversed.
    CorridorExtractionState state(width, height);

    for (voronoi_const_iterator it = points.begin(); it != points.end(); ++it)
    {
        PointID center = it->center;
        state.voronoiMap.insert( make_pair(center, it) );
        state.graph.setValue(center.x, center.y, 2);
    }

    while (!state.voronoiMap.empty())
    {
        PointID start_point = state.front().center;
        list<VoronoiPoint> cur_line;
        lineOrderingDFS(start_point, GridGraph::DIR_ALL, cur_line, state);
        if (cur_line.size() > 1)
            state.addBranch( start_point, cur_line );
    }

    state.branches.swap(branches);
}

void SkeletonExtraction::buildPlan(Plan& result, std::list<VoronoiPoint> const& points)
{
}

//void SkeletonExtraction::buildPixelMap(Plan& result) const
//{
//    // Build the pixel-to-corridor map. In there, "0" means "multiple owners"
//    // (i.e. crossroads)
//    result.pixel_map.resize(width * height);
//    int const NO_OWNER = result.corridors.size();
//    int const MULTIPLE_OWNERS = 0;
//    fill(result.pixel_map.begin(), result.pixel_map.end(), NO_OWNER);
//    //cerr << "Pixel map (border)\n";
//    for (size_t corridor_idx = 1; corridor_idx < result.corridors.size(); ++corridor_idx)
//    {
//        MedianPoint::BorderList const& borders = result.corridors[corridor_idx].borders;
//
//        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
//            for (PointVector::const_iterator point = it->begin(); point != it->end(); ++point)
//            {
//                uint8_t& owner = result.pixel_map[point->x + point->y * width];
//                if (owner == NO_OWNER)
//                    owner = corridor_idx;
//                else
//                    owner = MULTIPLE_OWNERS;
//                //cerr << corridor_idx << " " << (int)owner << " " << point->x << " " << point->y << endl;
//            }
//    }
//
//    /** Now, take each pixel and assign its corridor based on its parents */
//    //cerr << "Pixel map (interior)\n";
//    for (int idx = 0; idx < width * height; ++idx)
//    {
//        if (result.pixel_map[idx] != NO_OWNER)
//            continue;
//
//        ParentMap::const_iterator borders_it = parents.find(const_cast<height_t*>(&heightmap[idx]));
//        if (borders_it == parents.end())
//            continue;
//
//        int owner = NO_OWNER;
//        MedianPoint::BorderList const& borders = borders_it->second.borders;
//        for (MedianPoint::BorderList::const_iterator it = borders.begin(); it != borders.end(); ++it)
//            for (PointVector::const_iterator point = it->begin(); point != it->end(); ++point)
//            {
//                int border_owner = result.pixel_map[point->x + point->y * width];
//                if (owner == NO_OWNER)
//                {
//                    if (border_owner != MULTIPLE_OWNERS)
//                        owner = border_owner;
//                }
//                else if (border_owner != owner && border_owner != MULTIPLE_OWNERS)
//                    owner = MULTIPLE_OWNERS;
//                //cerr << (int)owner << " " << idx%width
//                    //<< " " << idx/width <<  " " << (int)border_owner << " " 
//                    //<< point->x << " " << point->y << endl;
//            }
//
//        result.pixel_map[idx] = owner;
//    }
//}

