rock_executable(fix_terrain_classes fix_terrain_classes.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)
