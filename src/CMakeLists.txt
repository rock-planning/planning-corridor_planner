rock_library(corridor_planner
    SOURCES
        voronoi.cc skeleton.cc plan.cc corridor_planner.cc annotations.cc
    DEPS_PKGCONFIG base-lib envire nav_graph_search
    DEPS_CMAKE     GDAL
    HEADERS
        merge.hh plan.hh skeleton.hh voronoi.hh
        corridor_planner.hh corridors.hh annotations.hh)

rock_executable(plan_corridors main.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)

rock_executable(fix_terrain_classes fix_terrain_classes.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)

