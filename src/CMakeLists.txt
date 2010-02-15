find_package(GDAL REQUIRED)
find_package(SISL REQUIRED)

add_definitions(${GDAL_CFLAGS})
include_directories(${GDAL_INCLUDE_DIRS} ${SISL_INCLUDE_DIRS})
add_library(Nav SHARED NURBSCurve3D.cc dstar.cc voronoi.cc skeleton.cc plan.cc)
target_link_libraries(Nav ${SISL_LIBRARIES} ${GDAL_LIBRARIES})

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
include_directories(${BASE_TYPES_INCLUDE_DIRS})
add_executable(nav nav.cc)
target_link_libraries(nav Nav)

add_executable(fix_terrain_classes fix_terrain_classes.cc)
target_link_libraries(fix_terrain_classes Nav)

install(TARGETS Nav
    LIBRARY DESTINATION lib)
install(FILES merge.hh plan.hh skeleton.hh voronoi.hh point.hh
    DESTINATION include/nav)

