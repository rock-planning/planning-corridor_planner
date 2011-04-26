ANN_UNKNOWN     = 0
ANN_STRONG_EDGE = 1

Typelib.specialize '/corridors/Corridor_m' do
    Annotations      = self['annotations'].deference.deference
    AnnotatedSegment = Annotations.deference

    def reverse
        self.median_curve.reverse

        # Reverse the curve an
        2.times do |i|
            self.boundary_curves[i].reverse
        end
        self.boundary_curves = self.boundary_curves.reverse
    end

    def join(corridor, geometric_resolution = 0.1, is_endpoint = false)
        # Offsets of the joined curves. Used at the end of the method to join
        # the annotations as well
        offsets = []
        # The width curve is a bit special. The constraint is that its
        # parametrization must match the parametrization of the median curve.
        #
        # We must join it ourselves, to maintain the constraint ...
        current_end = median_curve.end_param
        new_curve_length = (corridor.median_curve.end_param - corridor.median_curve.start_param)
        offsets[0] = median_curve.join(corridor.median_curve, geometric_resolution, false)

        interpolator_length = (median_curve.end_param - median_curve.start_param) - (current_end + new_curve_length)
        if interpolator_length == 0
            if is_endpoint && self.width_curve.singleton?
                self.width_curve = corridor.width_curve
            elsif !is_endpoint
                # Just did #append
                width_curve.append(corridor.width_curve)
            end
        else
            # Must create an interpolation segment (a segment is a good
            # approximation as we join all curves with segments)
            interpolator = Types::Base::Geometry::Spline.interpolate(
                 [width_curve.get(width_curve.end_param), corridor.width_curve.get(corridor.width_curve.start_param)],
                 [0, interpolator_length])
            width_curve.append(interpolator)
            width_curve.append(corridor.width_curve)
        end

        offsets[1] = boundary_curves[0].join(corridor.boundary_curves[0], geometric_resolution, false)
        offsets[2] = boundary_curves[1].join(corridor.boundary_curves[1], geometric_resolution, false)

        self_curves = [median_curve, boundary_curves[0], boundary_curves[1]]
        other_curves = [corridor.median_curve, corridor.boundary_curves[0], corridor.boundary_curves[1]]
        offsets.each_with_index do |off, idx|
            self_c  = self_curves[idx]
            other_c = other_curves[idx]

            self_ann  = annotations[idx]
            other_ann = corridor.annotations[idx]

            other_ann.each_with_index do |segments, annotation_idx|
                self_segments = []
                segments.each do |seg|
                    seg = seg.dup
                    seg.start += off
                    seg.end += off
                    self_segments << seg
                end

                if annotation_idx == self_ann.size
                    self_ann.push(self_segments)
                else
                    self_ann[annotation_idx].concat(self_segments)
                end
            end
        end
    end


    # Filters the given annotation information
    #
    # +operations+ is a mapping about geometrical constraints that are applied
    # on the filtered annotations. It is a mapping of the form
    #
    #   symbol => [min_width, max_hole]
    #
    # Where +min_width+ is the minimum width an annotated segment should have
    # without being removed and +max_hole+ the size between two consecutive
    # segments below which they get merged (if they have the same symbol,
    # obviously)
    def cleanup_single_annotations(curve, annotations, operations)
        filtered = []
        if annotations.empty?
            return filtered
        end

        current  = annotations.first
        for an in annotations[1, annotations.size - 1]
            min_width, max_hole = operations[an.symbol]

            do_push = true
            if max_hole && current.symbol == an.symbol
                if curve.length(current.end, an.start, 0.01) < max_hole
                    current.end = an.end
                    do_push = false
                end
            end

            if do_push
                if !min_width || curve.length(current.start, current.end, 0.01) > min_width
                    filtered << current
                end
                current = an
            end
        end

        if current
            if min_width && curve.length(current.start, current.end, 0.01) > min_width
                filtered << current
            end
        end
        filtered
    end

    # If curve is one of the boundary curves and +t+ a parameter on it, returns
    # the parameter of the associated median point
    #
    # Note that this point is *not* the closest median point. It is the median
    # point for which curve.get(t) is the closest point in +curve+.
    def associated_median_parameter(curve, t, geometric_resolution)
        median_curve.dichotomic_search(0.05) do |median_t, median_p|
            points, segments = curve.find_closest_points(median_p, geometric_resolution)

            if (points.empty? || points.last < t) && (segments.empty? || segments.last < t)
                true
            elsif (points.empty? || points.first > t) && (segments.empty? || segments.first > t)
                false
            else
                nil
            end
        end
    end

    # Returns corridor segments that have a given common symbol
    #
    # The returned value is an array of parameter intervals on the median curve
    def annotate_segments(annotation_index, symbol, save_as = nil)
        median_segments = [[], []]
        2.times do |curve_idx|
            curve = boundary_curves[curve_idx]
            annotations[curve_idx + 1][annotation_index].each do |segment|
                next if segment.symbol != symbol

                median_start = associated_median_parameter(curve, segment.start, 0.05)
                median_end   = associated_median_parameter(curve, segment.end, 0.05)
                median_segments[curve_idx] << [median_start, median_end]
            end
        end

        # Now compute segment intersections between the two median_segments sets
        median0, median1 = median_segments[0], median_segments[1]
        result = []
        median0.each do |seg_start, seg_end|
            while !median1.empty? && median1[0][0] > seg_end
                median1.shift
            end
            if median1.empty?
                break
            end

            if median1[0][1] < seg_start
                next
            end

            # So, now
            #   median1[0][0] < seg_end
            #   median1[0][1] > seg_start
            #
            # I.e. we have an intersection
            result << [[seg_start, median1[0][0]].max, [seg_end, median1[0][1]].min]
        end

        if save_as
            save_corridor_segments_as_annotation(save_as, symbol, result)
        end
        return result
    end

    def save_corridor_segments_as_annotation(annotation_idx, symbol, segments)
        segments = segments.map do |first, last|
            result = AnnotatedSegment.new
            result.start = first
            result.end = last
            result.symbol = symbol
            result
        end
        annotations[0][annotation_idx] = segments
    end

    def cleanup_annotations(index, operations)
        annotations[0][index] = cleanup_single_annotations(median_curve, annotations[0][index], operations)
        annotations[1][index] = cleanup_single_annotations(boundary_curves[0], annotations[1][index], operations)
        annotations[2][index] = cleanup_single_annotations(boundary_curves[1], annotations[2][index], operations)
    end
end

Typelib.specialize '/corridors/Plan_m' do
    # Returns the corridor object matching the given index
    def [](corridor_idx)
        corridors[corridor_idx]
    end

    CORRIDOR_OTHER_SIDE = { :FRONT_SIDE => :BACK_SIDE, :BACK_SIDE => :FRONT_SIDE }

    # Given a corridor and the side of the corridor we went in it, returns
    # the set of connections that can be used
    def next_corridors(last_corridor, last_in_side)
        last_out_side = CORRIDOR_OTHER_SIDE[last_in_side]
        connections.find_all do |conn|
            conn.from_idx == last_corridor && conn.from_side == last_out_side
        end
    end

    def each_path(current = Array.new, corridor_idx = nil, corridor_side = nil, seen = Set.new, &block)
        if !block_given?
            return enum_for(:each_path, current, corridor_idx, corridor_side)
        end

        if corridors.empty?
            return []
        end

        corridor_idx  ||= start_corridor
        corridor_side ||= :FRONT_SIDE

        current.push([corridor_idx, corridor_side])
        if corridor_idx == end_corridor
            yield(current.dup)
            return
        end

        outbound_connections = next_corridors(corridor_idx, corridor_side)
        if outbound_connections.empty?
            puts "WARN: found what it seems to be a dead end: #{current.inspect}"
            return
            # raise InternalError, "reached a dead end !!!"
        end

        outbound_connections.each do |conn|
            signature = [conn.from_idx, conn.from_side, conn.to_idx, conn.to_side]
            next if seen.include?(signature)

            begin
                seen << signature
                each_path(current, conn.to_idx, conn.to_side, seen, &block)
            ensure seen.delete(signature)
            end
        end

    ensure current.pop
    end

    def all_paths
        each_path.to_a
    end

    # Create a new Corridor object that represents the corridor path +path+
    def path_to_corridor(path)
        result = self.class['corridors'].deference.new
        result.zero!
        result.width_curve  = Types::Base::Geometry::Spline.new(1)
        result.median_curve = Types::Base::Geometry::Spline3.new
        result.boundary_curves[0] = Types::Base::Geometry::Spline3.new
        result.boundary_curves[1] = Types::Base::Geometry::Spline3.new

        endpoints = [path[0].first, path[-1].first]
        path.each do |idx, side|
            corridor = corridors[idx]
            if side == :BACK_SIDE
                corridor = corridor.dup
                corridor.reverse
            end

            result.join(corridor, 0.1, endpoints.include?(idx))
        end

        result
    end

    CORRIDOR_RECORD_NAMES = { :FRONT_SIDE => 0, :BACK_SIDE => 1 }
    def to_dot
        result = []
        result << "digraph {"
        result << "  rankdir=LR;"
        result << "  node [shape=record,height=.1];"
        connections.each do |connection|
            from_side = CORRIDOR_RECORD_NAMES[connection.from_side]
            to_side   = CORRIDOR_RECORD_NAMES[connection.to_side]
            result << "   c#{connection.from_idx}:#{from_side} -> c#{connection.to_idx}:#{to_side}"
        end
        corridors.size.times do |i|
            result << "   c#{i} [label=\"{<0>|<main> #{i}|<1>}\"];"
        end

        result << "};"
        result.join("\n")
    end

    def pretty_print(pp)
        pp.text "start: #{start_corridor}"
        pp.breakable
        pp.text "target: #{end_corridor}"
        pp.breakable
        corridors.each_with_index do |c, i|
            pp.text "#{i}: width=[#{c.min_width}, #{c.max_width}]"
            pp.breakable
        end

        connections.pretty_print(pp)
    end

    # Returns the index of the given annotation
    #
    # If +create+ is true, it will be added if it can't be found. Otherwise,
    # raises ArgumentError
    def find_annotation(symbol, create = false)
        ann_idx = annotation_symbols.index(symbol)
        if ann_idx
            return ann_idx
        elsif !create
            raise ArgumentError, "there is no annotations for the requested symbol #{symbol}. Known annotations are: #{annotation_symbols.to_a.join(", ")}"
        end

        ann_idx = annotation_symbols.size
        annotation_symbols.push(symbol)

        # Add a new annotation vector to all corridors as well
        corridors.each do |c|
            c.annotations[0].push(Annotations.new)
            c.annotations[1].push(Annotations.new)
            c.annotations[2].push(Annotations.new)
        end
        return ann_idx
    end

    def annotate_corridor_segments(annotation_symbol, symbol, save_as = nil)
        ann_idx = find_annotation(annotation_symbol)
        if save_as
            save_as_idx = find_annotation(save_as, true)
        end

        for c in corridors
            c.annotate_segments(ann_idx, symbol, save_as_idx)
        end
    end

    def cleanup_annotations(symbol, operations)
        ann_idx = find_annotation(annotation_symbol)
        for c in corridors
            c.cleanup_annotations(ann_idx, operations)
        end
    end
end

