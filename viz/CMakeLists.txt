include(RockRuby)
rock_ruby_library(corridor_planner corridor_plan_view.rb corridor_plan_view.ui)
rock_vizkit_plugin(corridor_planner-viz
    CorridorPlanVisualization.cpp
    DEPS corridor_planner
    DEPS_PKGCONFIG envire base-lib
    HEADERS CorridorPlanVisualization.hpp)
