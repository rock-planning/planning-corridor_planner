CONFIGURE_FILE(testsuite.hh.in ${CMAKE_BINARY_DIR}/test/testsuite.hh @ONLY)

include_directories(${PROJECT_SOURCE_DIR}/src)

ADD_EXECUTABLE(dstar_testsuite testsuite.cc test_dstar.cc test_dstar_multipath.cc)
TARGET_LINK_LIBRARIES(dstar_testsuite Nav ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
ADD_TEST(CxxSuite ${EXECUTABLE_OUTPUT_PATH}/dstar_testsuite)

ADD_EXECUTABLE(skeleton_testsuite testsuite.cc test_corridors.cc)
TARGET_LINK_LIBRARIES(skeleton_testsuite Nav ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
ADD_TEST(CxxSuite ${EXECUTABLE_OUTPUT_PATH}/skeleton_testsuite)

