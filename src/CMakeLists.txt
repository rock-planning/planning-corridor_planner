rock_library(corridor_planner
    SOURCES
        terrain_classes.cc traversability_map.cc grid_graph.cc
        dstar.cc voronoi.cc skeleton.cc plan.cc corridor_planner.cc
#        traversability_classifier.cc
    DEPS_PKGCONFIG base-lib envire
    DEPS_CMAKE     GDAL
    HEADERS
        terrain_classes.hh grid_map.hh traversability_map.hh grid_graph.hh
        point.hh geometry.hh
        dstar.hh merge.hh plan.hh skeleton.hh voronoi.hh
        corridor_planner.hh pool_allocator.hh corridors.hh)
#        traversability_classifier.hh)

rock_executable(plan_corridors main.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)

rock_executable(fix_terrain_classes fix_terrain_classes.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)

# rock_executable(make_traversability make_traversability.cc
#     DEPS corridor_planner
#     DEPS_PKGCONFIG base-types)

# rock_executable(make_band_from_classes make_band_from_classes.cc
#     DEPS corridor_planner
#     DEPS_PKGCONFIG base-types)


