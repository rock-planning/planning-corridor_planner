ANN_UNKNOWN     = 0
ANN_STRONG_EDGE = 1

Typelib.specialize_model '/corridors/Plan_m' do
    def from_path(path)
        result = self.new
        result.zero!
        result.start_corridor = 0
        result.end_corridor = path.size - 1
        path.each_with_index do |(corridor, modality), idx|
            result.corridors << corridor
            if idx != 0
                conn = Types::Corridors::CorridorConnection.new
                conn.zero!
                conn.from_idx = idx - 1
                conn.from_side = :BACK_SIDE
                conn.to_idx   = idx
                conn.to_side = :FRONT_SIDE
                result.connections << conn
            end
        end
        result
    end
end

Typelib.specialize '/corridors/Corridor_m' do
    Annotations      = self['annotations'].deference.deference
    AnnotatedSegment = Annotations.deference

    # Reverses the direction of this corridor
    def reverse
        curves = [median_curve, boundary_curves[0], boundary_curves[1]]
        old_start_t = []
        curves.each_with_index do |curve, curve_idx|
            old_start_t = curve.start_param
            length = curve.end_param - curve.start_param
            curve.reverse
            new_start_t = curve.start_param
            self.annotations[curve_idx] = annotations[curve_idx].map do |per_symbol|
                result = []
                per_symbol.each do |ann|
                    start, finish = ann.start, ann.end
                    ann.start = length - (finish - old_start_t) + new_start_t
                    ann.end   = length - (start - old_start_t) + new_start_t
                    result.unshift(ann)
                end
                result
            end
        end

        self.boundary_curves = self.boundary_curves.reverse
        temp = self.annotations[1].dup
        self.annotations[1] = self.annotations[2]
        self.annotations[2] = temp
        self.width_curve.reverse
    end

    # Helper method for #split
    def split_single_annotation(annotations, pos)
        old_annotations, new_annotations = annotations.partition do |seg|
            seg.end <= pos
        end
        if !new_annotations.empty? && new_annotations.first.start < pos
            old_annotations << new_annotations.first.dup
            old_annotations.last.end = pos
            new_annotations.first.start = pos
        end
        return old_annotations, new_annotations
    end

    def split_annotations(annotations, pos)
        old, new = [], []
        annotations.each do |ann|
            old_ann, new_ann = split_single_annotation(ann, pos)
            old << old_ann
            new << new_ann
        end
        return old, new
    end

    # Splits the corridor at the given position, where +position+ is a parameter
    # on the median curve
    def split(position)
        new_corridor = self.class.new
        new_corridor.zero!
        new_corridor.min_width = min_width
        new_corridor.max_width = max_width

        result = median_curve.split(position)
        new_corridor.median_curve = result

        new_corridor.width_curve  = width_curve.split(position)
        old_annotations, new_annotations = split_annotations(annotations[0], position)
        self.annotations[0] = old_annotations
        new_corridor.annotations[0] = new_annotations

        2.times do |boundary_idx|
            boundary_pos = associated_boundary(position, boundary_idx, 0.01).inject(&:+) / 2
            result = boundary_curves[boundary_idx].split(boundary_pos)
            new_corridor.boundary_curves[boundary_idx] = result

            old_annotations, new_annotations = split_annotations(annotations[boundary_idx + 1], position)
            self.annotations[boundary_idx + 1] = old_annotations
            new_corridor.annotations[boundary_idx + 1] = new_annotations
        end

        new_corridor
    end

    # Splits this corridor following the annotations for +annotation_idx+
    def split_annotation_segments(annotation_idx)
        segments = annotations[0][annotation_idx]
        if segments.empty?
            return [[nil, self]]
        end

        current_corridor = self
        result = [[nil, current_corridor]]
        last_pos = median_curve.start_param
        segments.each do |seg|
            new_pos = seg.start
            if new_pos == median_curve.start_param
                result[0][0] = seg.symbol
            elsif last_pos != median_curve.start_param
                current_corridor = current_corridor.split(last_pos)
                result << [nil, current_corridor]
            end
            if new_pos != last_pos
                current_corridor = current_corridor.split(new_pos)
                result << [seg.symbol, current_corridor]
            end
            last_pos = seg.end
        end
        if last_pos != median_curve.end_param
            current_corridor = current_corridor.split(last_pos)
            result << [nil, current_corridor]
        end

        result
    end

    def join(corridor, geometric_resolution = 0.1)
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

        if interpolator_length <= 0
            width_curve.append(corridor.width_curve)
        else
            # Must create an interpolation segment (a segment is a good
            # approximation as we join all curves with segments)
            interpolator = Types::Base::Geometry::Spline.interpolate(
                 [width_curve.get(width_curve.end_param), corridor.width_curve.get(corridor.width_curve.start_param)],
                 [0, interpolator_length])
            width_curve.append(interpolator)
            width_curve.append(corridor.width_curve)
        end

        if self.min_width < 0
            self.min_width = corridor.min_width
            self.max_width = corridor.max_width
        else
            if self.min_width > corridor.min_width
                self.min_width = corridor.min_width
            end
            if self.max_width < corridor.max_width
                self.max_width = corridor.max_width
            end
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
            min_width, max_hole = operations[current.symbol]

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
            min_width, max_hole = operations[current.symbol]
            if !min_width || curve.length(current.start, current.end, 0.01) > min_width
                filtered << current
            end
        end
        filtered
    end

    # Returns the region of the requested boundary curve that is associated with
    # the given point on the median curve.
    def associated_boundary(t, boundary_idx, geometric_resolution)
        median_p = median_curve.get(t)
        curve = boundary_curves[boundary_idx]
        points, segments = curve.find_closest_points(median_p, geometric_resolution)
        
        min, max = curve.end_param, curve.start_param
        if !points.empty?
            min = [min, points.min].min
            max = [max, points.max].max
        end
        if !segments.empty?
            min = [min, segments.min[0]].min
            max = [max, segments.max[1]].max
        end
        return [min, max]
    end

    # If curve is one of the boundary curves and +t+ a parameter on it, returns
    # the parameter of the associated median point
    #
    # Note that this point is *not* the closest median point. It is the median
    # point for which curve.get(t) is the closest point in +curve+.
    def associated_median_parameter(curve, t, geometric_resolution)
        median_curve.dichotomic_search(0.05) do |median_t, median_p|
            points, segments = curve.find_closest_points(median_p, geometric_resolution)

            if (points.empty? || points.last < t) && (segments.empty? || segments.last.last < t)
                true
            elsif (points.empty? || points.first > t) && (segments.empty? || segments.first.first > t)
                false
            else
                nil
            end
        end
    end

    def do_segment_intersection(median0, median1) # :nodoc:
        # Now compute segment intersections between the two median_segments sets
        result = []
        while !median0.empty? && !median1.empty?
            while !median0.empty? && median0[0][1] < median1[0][0] 
                median0.shift
            end
            break if median0.empty?

            seg_start, seg_end = median0[0]
            while !median1.empty? && median0[0][0] > median1[0][1]
                median1.shift
            end
            break if median1.empty?

            if median0[0][1] < median1[0][0]
                next
            end

            # So, now
            #   median1[0][0] < seg_end
            #   median1[0][1] > seg_start
            #
            # I.e. we have an intersection
            result << [[seg_start, median1[0][0]].max, [seg_end, median1[0][1]].min]
            median1.shift
        end
        result
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

        median0, median1 = median_segments[0], median_segments[1]
        result = do_segment_intersection(median0, median1)

        if save_as
            save_corridor_segments_as_annotation(0, save_as, symbol, result)
        end
        return result
    end

    def save_corridor_segments_as_annotation(curve_idx, annotation_idx, symbol, segments)
        segments = segments.map do |first, last|
            result = AnnotatedSegment.new
            result.start = first
            result.end = last
            result.symbol = symbol
            result
        end
        annotations[curve_idx][annotation_idx] = segments
    end

    def intersect_annotations(ann0_idx, symbol0, ann1_idx, symbol1, save_as)
        result = []

        curves = [median_curve, boundary_curves[0], boundary_curves[1]]
        curves.each_with_index do |curve, curve_idx|
            segments0 = []
            last_end = curve.start_param
            annotations[curve_idx][ann0_idx].each do |seg|
                if !symbol0
                    segments0 << [last_end, seg.start]
                    last_end = seg.end
                elsif seg.symbol == symbol0
                    segments0 << [seg.start, seg.end]
                end
            end
            if !symbol0
                segments0 << [last_end, curve.end_param]
            end

            segments1 = []
            last_end = curve.start_param
            annotations[curve_idx][ann1_idx].each do |seg|
                if !symbol1
                    segments1 << [last_end, seg.start]
                    last_end = seg.end
                elsif seg.symbol == symbol1
                    segments1 << [seg.start, seg.end]
                end
            end
            if !symbol1
                segments1 << [last_end, curve.end_param]
            end

            result << do_segment_intersection(segments0, segments1)
            if save_as
                save_corridor_segments_as_annotation(curve_idx, save_as, 0, result.last)
            end
        end

        result
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
    def path_to_corridor(path, start_end_width = 0.5)
        path = path.dup
        path_corridors = path.map do |idx, side|
            corridor = corridors[idx]
            if side == :BACK_SIDE
                corridor = corridor.dup
                corridor.reverse
            end
            corridor
        end

        if path.size == 1
            return path_corridors[0].dup
        end

        result = self.class['corridors'].deference.new
        result.zero!
        result.min_width = -1
        result.max_width = -1
        result.width_curve  = Types::Base::Geometry::Spline.new(1)
        result.median_curve = Types::Base::Geometry::Spline3.new
        result.boundary_curves = [Types::Base::Geometry::Spline3.new, Types::Base::Geometry::Spline3.new]

        # Check if the endpoints "touch" the corridors. If it is the case, just
        # remove them. Otherwise, we transform them so that the "last mile" is
        # not too narrow
        endpoint0 = path_corridors[0]
        corridor0 = path_corridors[1]
        if (endpoint0.median_curve.end_point - corridor0.median_curve.start_point).norm < 0.1
            path_corridors.shift
        else
            u = (corridor0.boundary_curves[1].start_point - corridor0.boundary_curves[0].start_point).
                normalize * start_end_width
            path_corridors[0] = endpoint0 = endpoint0.dup
            endpoint0.boundary_curves[1] = Types::Base::Geometry::Spline3.singleton(endpoint0.median_curve.end_point + u)
            endpoint0.boundary_curves[0] = Types::Base::Geometry::Spline3.singleton(endpoint0.median_curve.end_point - u)
            endpoint0.width_curve = Types::Base::Geometry::Spline.singleton([start_end_width * 2])
        end

        corridor1 = path_corridors[-2]
        endpoint1 = path_corridors[-1]
        if (endpoint1.median_curve.start_point - corridor1.median_curve.end_point).norm < 0.1
            path_corridors.shift
        else
            u = (corridor1.boundary_curves[1].start_point - corridor1.boundary_curves[0].start_point).
                normalize * start_end_width
            path_corridors[-1] = endpoint1 = endpoint1.dup
            endpoint1.boundary_curves[1] = Types::Base::Geometry::Spline3.singleton(endpoint1.median_curve.start_point + u)
            endpoint1.boundary_curves[0] = Types::Base::Geometry::Spline3.singleton(endpoint1.median_curve.start_point - u)
            endpoint1.width_curve = Types::Base::Geometry::Spline.singleton([start_end_width * 2])
        end

        path_corridors.each do |c|
            result.join(c, 0.1)
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

    # Computes annotations on the median curve for regions of the corridors that
    # have matching symbols on both boundaries.
    #
    # If +save_as+ is non-nil, the result is saved as a median curve annotation
    # with the symbol name given by +save_as+ (which can be the same than
    # +annotation_symbol+.
    #
    # The result is returned as an array of the form
    #
    #   result = [
    #      [[start, stop], [start, stop], ...],
    #      [[start, stop], [start, stop], ...],
    #      ...
    #   ]
    #
    # where each [start, stop] pair is a segment on the median curve of the
    # corresponding corridor
    #
    # I.e. result[0] lists the segments on corridors[0].median_curve
    def annotate_corridor_segments(annotation_symbol, symbol, save_as = nil)
        ann_idx = find_annotation(annotation_symbol)
        if save_as
            save_as_idx = find_annotation(save_as, true)
        end

        result = []
        corridors.each_with_index do |c, i|
            result << c.annotate_segments(ann_idx, symbol, save_as_idx)
        end
        result
    end

    # Intersects two given annotations
    #
    # If +save_as+ is non-nil, the result is saved as an annotation with the
    # symbol name given by +save_as+ (which can be the same than
    # +annotation_symbol+.
    #
    # The result is returned as an array of the form
    #
    #   result = [
    #     corridor0 = [
    #       [[start, stop], [start, stop], ...],
    #       [[start, stop], [start, stop], ...],
    #       [[start, stop], [start, stop], ...]
    #     ],
    #     corridor1 = ...
    #   ]
    #
    def intersect_annotations(ann0, symbol0, ann1, symbol1, save_as = nil)
        ann0_idx = find_annotation(ann0)
        ann1_idx = find_annotation(ann1)
        if save_as
            save_as_idx = find_annotation(save_as, true)
        end

        result = []
        corridors.each_with_index do |c, i|
            result << c.intersect_annotations(ann0_idx, symbol0, ann1_idx, symbol1, save_as_idx)
        end
        result
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
    def cleanup_annotations(symbol, operations)
        ann_idx = find_annotation(symbol)
        for c in corridors
            c.cleanup_annotations(ann_idx, operations)
        end
    end
end

