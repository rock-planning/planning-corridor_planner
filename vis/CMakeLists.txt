pkg_check_modules(envire REQUIRED envire)
include_directories(${envire_INCLUDE_DIRS})
include_directories(corridor_planner_vis ${enview_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_library(corridor_planner_vis SHARED CorridorPlanVisualisation.cpp)
target_link_libraries(corridor_planner_vis ${enview_LIBRARIES}
    ${envire_LIBRARIES})
install(TARGETS corridor_planner_vis
    DESTINATION lib)
install(FILES CorridorPlanVisualisation.h
    DESTINATION include/enview)

