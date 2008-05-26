require 'Tioga/FigureMaker'
require 'stringio'
include Tioga

class Plotter
    include Tioga::FigureConstants

    attr_reader :t
    attr_reader :data
    def initialize(io)
        @t = Tioga::FigureMaker.default

        @data = io.read
        t.def_figure("Graph") { dstar_graph }
        t.def_figure("Costs") { dstar_costs }
    end

    def read_header(io)
        values = io.readline.split(" ")
        if values.size == 4
            x_start, y_start, x_goal, y_goal = Float(values[0]), Float(values[1]), Float(values[2]), Float(values[3])
            values = io.readline.split(" ")
        end
        x_size, y_size = Float(values[0]), Float(values[1])
        return x_size, y_size, x_start, y_start, x_goal, y_goal
    end

    def dstar_graph
        t.line_width = 0

        file = StringIO.new data
        x_size, y_size, x_start, y_start, x_goal, y_goal = read_header file

        file.each_line do |line|
            if line == "\n"
                break
            end

            values = line.split " "
            if values.size == 5
                x0, y0 = Float(values[0]), Float(values[1])
                x1, y1 = Float(values[3]), Float(values[4])
                t.stroke_line((0.5+x0)/x_size, (0.5+y0)/y_size, (0.5+x1)/x_size, (0.5+y1)/y_size)
            end
        end

        if x_start
            t.show_marker 'at' => [(0.5+x_start) / x_size, (0.5+y_start) / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Red
        end
        if x_goal
            t.show_marker 'at' => [(0.5+x_goal) / x_size, (0.5+y_goal) / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Red
        end
    end

    def dstar_costs
        max = nil
        t.subplot('right_margin' => 0.07) { max = cost_image }
        t.subplot('left_margin' => 0.95,
                  'top_margin' => 0.05,
                  'bottom_margin' => 0.05) { gradient_bar("Cost", 0, max, t.rainbow_colormap) }
    end

    def cost_image
        t.line_width = 0

        file = StringIO.new data
        x_size, y_size, x_start, y_start, x_goal, y_goal = read_header file

        costs = Dtable.new(x_size, y_size)
        contour_mode = false
        contour = []
        parents = Hash.new
        file.each_line do |line|
            if line == "\n"
                contour_mode = true
                next
            end

            values = line.split " "
            x0, y0 = Integer(values[0]), Integer(values[1])
            if contour_mode
                contour << [x0, y0]
            else
                costs[y_size - 1 - y0, x0] = Float(values[2])
                if values[3]
                    x1, y1 = Integer(values[3]), Integer(values[4])
                    parents[[x0, y0]] = [x1, y1]
                end
            end
        end
        max = costs.max
        image = t.create_image_data(costs, 'min_value' => 0, 'max_value' => max)

        t.fill_color = Wheat
        t.fill_frame
        t.show_image(
                'll' => [0, 0],
                'lr' => [1, 0],
                'ul' => [0, 1],
                'color_space' => t.rainbow_colormap,
                'data' => image,
                'w' => x_size, 'h' => y_size)

        contour.each do |x, y|
            p = parents[[x, y]]
            if p
                t.stroke_line((x+0.5)/x_size, (y+0.5)/y_size, (p[0]+0.5)/x_size, (p[1]+0.5)/y_size)
            end
        end
        if x_start
            t.show_marker 'at' => [(x_start+0.5) / x_size, (y_start+0.5) / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Green
            t.show_marker 'at' => [(x_goal+0.5) / x_size, (y_goal+0.5) / y_size],
                'marker' => Bullet, 'scale' => 0.5, 'color' => Green
        end
        max
    end

    def gradient_bar(title, min, max, colormap)
        xmin = 0; xmax = 1; xmid = 0.5
        t.rescale(0.8)
        t.xaxis_type = AXIS_LINE_ONLY
        t.xaxis_loc  = BOTTOM
        t.top_edge_type = AXIS_LINE_ONLY
        t.yaxis_loc = t.ylabel_side = RIGHT
        t.yaxis_type = AXIS_WITH_TICKS_AND_NUMERIC_LABELS
        t.left_edge_type = AXIS_WITH_TICKS_ONLY
        t.ylabel_shift += 0.5
        t.yaxis_major_tick_length *= 0.6
        t.yaxis_minor_tick_length *= 0.5
        t.do_box_labels(nil, nil, title)
        t.show_plot('boundaries' => [xmin, xmax, min, max]) do
            t.axial_shading(
                'start_point' => [xmid, min],
                'end_point'   => [xmid, max],
                'colormap'    => colormap
            )
        end
    end
end

