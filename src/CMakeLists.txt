find_package(GDAL)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GDAL_CFLAGS}")

add_library(Nav SHARED dstar.cc voronoi.cc skeleton.cc plan.cc merge.cc)
target_link_libraries(Nav ${GDAL_LIBRARIES})

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

