#ifndef NAV_TERRAIN_CLASSES_HH
#define NAV_TERRAIN_CLASSES_HH

#include <list>
#include <string>

namespace corridor_planner
{
    /** This structure is a terrain class as described in a on-disk map */
    struct TerrainClass
    {
        int in; //! The class value on disk
        int out; //! The class value to be used in TraversabilityMap
        float cost; //! The cost of this class
        float margin; //! The needed geometrical margin so that to use the maximum speed
        std::string name; //! The class name

        static std::list<TerrainClass> load(std::string const& path);
    };

    typedef std::list<TerrainClass> TerrainClasses;

}

#endif

