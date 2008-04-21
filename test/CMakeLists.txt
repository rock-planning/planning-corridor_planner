CONFIGURE_FILE(testsuite.hh.in test/testsuite.hh @ONLY)
ADD_EXECUTABLE(nav_testsuite testsuite.cc test_dstar.cc)

TARGET_LINK_LIBRARIES(nav_testsuite Nav ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
ADD_TEST(CxxSuite ${EXECUTABLE_OUTPUT_PATH}/nav_testsuite)

