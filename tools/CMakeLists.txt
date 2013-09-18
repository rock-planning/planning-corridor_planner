rock_executable(fix_terrain_classes fix_terrain_classes.cc
    DEPS corridor_planner
    DEPS_CMAKE GDAL
    DEPS_PKGCONFIG base-types)

rock_find_cmake(Boost)
rock_executable(env_augment_mls env_augment_mls.cpp
    DEPS_PKGCONFIG envire base-types base-lib
    DEPS_PLAIN Boost Boost_SYSTEM)
