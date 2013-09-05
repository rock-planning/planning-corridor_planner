configure_file(testsuite.hh.in ${CMAKE_CURRENT_BINARY_DIR}/testsuite.hh @ONLY)
include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

# rock_testsuite(dstar_testsuite
#     testsuite.cc test_dstar.cc test_dstar_multipath.cc
#     DEPS corridor_planner)

rock_testsuite(skeleton_testsuite
    testsuite.cc test_corridors.cc
    DEPS corridor_planner
    DEPS_PLAIN Boost_SYSTEM)

