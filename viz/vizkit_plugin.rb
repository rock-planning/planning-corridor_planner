Vizkit::UiLoader.register_3d_plugin('CorridorPlan3DPlugin', 'corridor_planner', 'CorridorPlanVisualization')
Vizkit::UiLoader.register_3d_plugin_for('CorridorPlan3DPlugin', "/corridors/Plan_m", :updatePlan)

require 'corridor_planner/corridor_plan_view'
Vizkit::UiLoader.register_ruby_widget("CorridorPlanView", CorridorPlanView.method(:new))
Vizkit::UiLoader.register_default_widget_for("CorridorPlanView","/corridors/Plan")


