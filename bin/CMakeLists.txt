FIND_PACKAGE(GDAL)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GDAL_CFLAGS}")

pkg_check_modules(DFKI_TYPES REQUIRED "dfki-types")

INCLUDE_DIRECTORIES(${DFKI_TYPES_INCLUDE_DIRS})
add_executable(dstar dstar.cc)
target_link_libraries(dstar Nav)

add_executable(fix_terrain_classes fix_terrain_classes.cc)
target_link_libraries(fix_terrain_classes Nav)

INSTALL(TARGETS dstar
    RUNTIME DESTINATION bin)

