Typelib.specialize '/corridors/Corridor_m' do
    def reverse
        self.median_curve.reverse

        # Reverse the curve an
        2.times do |i|
            self.boundary_curves[i].reverse
        end
        self.boundary_curves = self.boundary_curves.reverse
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

        corridor_idx  ||= start_corridor
        corridor_side ||= :FRONT_SIDE

        current.push([corridor_idx, corridor_side])
        if corridor_idx == end_corridor
            yield(current.dup)
            return
        end

        outbound_connections = next_corridors(corridor_idx, corridor_side)
        if outbound_connections.empty?
            raise InternalError, "reached a dead end !!!"
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
        median, *boundaries = (1..3).map { Types::Base::Geometry::Spline3.new(3) }

        # puts median.kind
        # puts boundaries.map(&:kind).joi(", ")

        path.each do |idx, side|
            corridor = corridors[idx]

            if side == :BACK_SIDE
                corridor = corridor.dup
                corridor.reverse
            end

            median.join(corridor.median_curve, 0.1)
            2.times do |i|
                boundaries[i].join(corridor.boundary_curves[i])
            end
            # puts "added corridor #{idx}"
            # puts "  " + corridor.median_curve.start_point.inspect
            # puts "  " + corridor.median_curve.end_point.inspect
            # puts "-- boundary[0]"
            # puts "  " + corridor.boundary_curves[0].start_point.inspect
            # puts "  " + corridor.boundary_curves[0].end_point.inspect
            # puts "-- boundary[1]"
            # puts "  " + corridor.boundary_curves[1].start_point.inspect
            # puts "  " + corridor.boundary_curves[1].end_point.inspect
        end

        corridor = self.class.corridors.deference.new
        corridor.median_curve = median
        corridor.boundary_curves = boundaries
        corridor
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
end
