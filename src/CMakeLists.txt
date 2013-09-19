rock_library(corridor_planner
    SOURCES
        voronoi.cc skeleton.cc plan.cc corridor_planner.cc annotations.cc
    DEPS_PKGCONFIG base-lib envire nav_graph_search
    DEPS_CMAKE     GDAL
    HEADERS
        merge.hh plan.hh skeleton.hh voronoi.hh
        corridor_planner.hh corridors.hh annotations.hh)

include(FindBoost)
rock_find_cmake(Boost COMPONENTS system)
rock_executable(plan_corridors main.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PLAIN Boost Boost_SYSTEM
    DEPS_PKGCONFIG base-types)
