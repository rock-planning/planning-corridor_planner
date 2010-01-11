CONFIGURE_FILE(testsuite.hh.in ${CMAKE_BINARY_DIR}/test/testsuite.hh @ONLY)
ADD_EXECUTABLE(nav_testsuite testsuite.cc test_dstar.cc test_dstar_multipath.cc)

TARGET_LINK_LIBRARIES(nav_testsuite Nav ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
ADD_TEST(CxxSuite ${EXECUTABLE_OUTPUT_PATH}/nav_testsuite)

