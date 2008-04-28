require 'Tioga/FigureMaker'
include Tioga

class Plotter
    include Tioga::FigureConstants

    attr_reader :t
    def initialize(path)
        @t = Tioga::FigureMaker.default

        t.def_figure("Paths") { plot_dstar_result(path) }
        t.def_figure("Costs") { plot_dstar_costs(path) }
    end

    def plot_dstar_result(path)
        t.line_width = 0
        File.open(path) do |file|
            values = file.readline.split(" ")
            x_size, y_size = Float(values[0]), Float(values[1])
            values = file.readline.split(" ")
            x_start, y_start, x_goal, y_goal = Float(values[0]), Float(values[1]), Float(values[2]), Float(values[3])

            t.show_marker 'at' => [x_start / x_size, y_start / y_size],
                'marker' => Bullet, 'scale' => 0.5
            t.show_marker 'at' => [x_goal / x_size, y_goal / y_size],
                'marker' => Bullet, 'scale' => 0.5
            file.each_line do |line|
                values = line.split " "
                x0, y0 = Float(values[0]), Float(values[1])
                x1, y1 = Float(values[3]), Float(values[4])
                t.stroke_line x0/x_size, y0/y_size, x1/x_size, y1/y_size
            end
        end
    end
end

plotter = Plotter.new 'ndd.txt'

