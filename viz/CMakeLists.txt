include_directories(${vizkit_INCLUDE_DIRS})
link_directories(${vizkit_LIBRARY_DIRS})

pkg_check_modules(envire REQUIRED envire)
include_directories(${envire_INCLUDE_DIRS})
link_directories(${envire_LIBRARY_DIRS})

pkg_check_modules(envire_viz REQUIRED envire-viz)
include_directories(${envire_viz_INCLUDE_DIRS})
link_directories(${envire_viz_LIBRARY_DIRS})

pkg_check_modules(base REQUIRED base-lib)
include_directories(${base_INCLUDE_DIRS})
link_directories(${base_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_library(corridor_planner-viz SHARED CorridorPlanVisualization.cpp)
target_link_libraries(corridor_planner-viz ${envire_LIBRARIES} ${envire_viz_LIBRARIES}
    ${vizkit_LIBRARIES} ${base_LIBRARIES})
install(TARGETS corridor_planner-viz
    DESTINATION lib)
install(FILES CorridorPlanVisualization.hpp
    DESTINATION include/vizkit)

configure_file(corridor_planner-viz.pc.in corridor_planner-viz.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/corridor_planner-viz.pc
    DESTINATION lib/pkgconfig)
