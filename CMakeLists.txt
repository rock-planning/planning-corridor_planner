PROJECT(corridor_planner)
cmake_minimum_required(VERSION 2.6)
SET(PROJECT_VERSION 0.1)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(FindPkgConfig)
include(RPATHHandling)
include(CTest)
include(GenerateDoxygenDoc)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})
cmake_use_full_rpath("${CMAKE_INSTALL_PREFIX}/lib")

pkg_check_modules(BASE_LIB REQUIRED "base-lib")
include_directories(${BASE_LIB_INCLUDE_DIRS})
link_directories(${BASE_LIB_LIBRARY_DIRS})

add_subdirectory(src)
configure_file(${CMAKE_SOURCE_DIR}/corridor_planner.pc.in
    ${CMAKE_BINARY_DIR}/corridor_planner.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/corridor_planner.pc
    DESTINATION lib/pkgconfig)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)
IF (Boost_UNIT_TEST_FRAMEWORK_FOUND)
    MESSAGE(STATUS "boost/test found ... building test suite")
    ADD_SUBDIRECTORY(test)
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

