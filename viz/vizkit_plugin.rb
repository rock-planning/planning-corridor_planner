Vizkit::UiLoader.register_3d_plugin('CorridorPlanVisualization', 'corridor_planner', 'CorridorPlanVisualization')
Vizkit::UiLoader.register_3d_plugin_for('CorridorPlanVisualization', "/corridors/Plan_m", :updatePlan)

require 'corridor_planner/corridor_plan_view'
Vizkit::UiLoader.register_ruby_widget("CorridorPlanView", CorridorPlanView.method(:new))
Vizkit::UiLoader.register_default_widget_for("CorridorPlanView","/corridors/Plan")


