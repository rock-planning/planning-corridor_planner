find_package(GDAL REQUIRED)

add_definitions(${GDAL_CFLAGS})
include_directories(${GDAL_INCLUDE_DIRS})

add_library(Nav SHARED
    terrain_classes.cc traversability_map.cc grid_graph.cc
    dstar.cc voronoi.cc skeleton.cc plan.cc corridor_planner.cc)
target_link_libraries(Nav ${GDAL_LIBRARIES} ${BASE_LIB_LIBRARIES})

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
include_directories(${BASE_TYPES_INCLUDE_DIRS})
add_executable(nav nav.cc)
target_link_libraries(nav Nav)

add_executable(fix_terrain_classes fix_terrain_classes.cc)
target_link_libraries(fix_terrain_classes Nav)

install(TARGETS Nav
    LIBRARY DESTINATION lib)
install(FILES terrain_classes.hh grid_map.hh traversability_map.hh grid_graph.hh
        point.hh geometry.hh
        dstar.hh merge.hh plan.hh skeleton.hh voronoi.hh
        corridor_planner.hh pool_allocator.hh
        DESTINATION include/nav)

