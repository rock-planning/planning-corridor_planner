FIND_PACKAGE(GDAL)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GDAL_CFLAGS}")

add_executable(dstar dstar.cc)
target_link_libraries(dstar Nav)

add_executable(fix_terrain_classes fix_terrain_classes.cc)
target_link_libraries(fix_terrain_classes Nav)

INSTALL(TARGETS dstar
    RUNTIME DESTINATION bin)

