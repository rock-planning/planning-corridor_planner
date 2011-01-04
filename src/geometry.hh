#ifndef NAV_GEOMETRY_TOOLS_HH
#define NAV_GEOMETRY_TOOLS_HH

#include <corridor_planner/point.hh>

namespace corridor_planner
{
    template<typename _It>
    _It findNearest(_It begin, _It end, PointID const& p)
    {
	if (begin == end)
	    return end;

	float min_distance = p.distance2(get_point(*begin));
	_It result = begin;
	for (_It it = ++begin; it != end; ++it)
	{
	    float d = p.distanceTo2(get_point(*it));
	    if (min_distance > d)
	    {
		min_distance = d;
		result = it;
	    }
	}
	return result;
    }

}

#endif

