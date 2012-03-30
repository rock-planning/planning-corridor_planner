Vizkit::UiLoader.register_3d_plugin('CorridorPlanViewer', 'corridor_planner', 'CorridorPlanVisualization')
Vizkit::UiLoader.register_3d_plugin_for('CorridorPlanViewer', "/corridors/Plan_m", :updatePlan)

require 'corridor_planner/corridor_plan_view'
Vizkit::UiLoader.register_ruby_widget("CorridorPlanView", CorridorPlanView.method(:new))
Vizkit::UiLoader.register_widget_for("CorridorPlanView","/corridors/Plan")


