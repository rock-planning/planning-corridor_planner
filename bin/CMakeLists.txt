FIND_PACKAGE(GDAL)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GDAL_CFLAGS}")

add_executable(dstar dstar.cc)
target_link_libraries(dstar Nav)

INSTALL(TARGETS dstar
    RUNTIME DESTINATION bin)

