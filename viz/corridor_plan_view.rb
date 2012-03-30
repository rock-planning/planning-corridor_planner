class CorridorPlanView < Qt::Widget
    attr_reader :plan
    attr_reader :all_paths
    attr_reader :vizkit_corridors
    attr_reader :view3d

    attr_reader :controls

    def initialize(view3d = nil, parent = nil)
        super(parent)
        if view3d
            @view3d = view3d
        else
            @view3d = Vizkit.vizkit3d_widget
            @view3d.show
        end
        setupUi
    end

    def update(plan, port = nil)
        controls.grpPath.setEnabled(true)
        controls.grpAnnotations.setEnabled(true)
        @plan = plan
        @show_all = false
        @all_paths = plan.all_paths
        vizkit_corridors.clearCorridors(0)
        if !@all_paths.empty?
            controls.status.setText("#{@all_paths.size} paths in plan")
            controls.pathIdx.setRange(-1, @all_paths.size - 1)
            setPath(-1)
        else
            controls.status.setText("Empty plan")
        end

        vizkit_corridors.updatePlan(plan)
        update_symbols
    end

    def export_current_corridor
        if @current_corridor
            @corridor_log ||= Pocolog::Logfiles.create('exported_corridors')
            @corridor_stream ||= @corridor_log.stream('ui.exported_corridors', Types::Corridors::Corridor, true)
            @corridor_stream.write(Time.now, Time.now, @current_corridor)
        end
    end

    def setPath(path_idx)
        controls.pathIdx.setValue(path_idx)
        vizkit_corridors.clearCorridors(0)

        if path_idx == -1
            if @show_all
                @all_paths.each do |p|
                    begin
                        corridor = plan.path_to_corridor(p)
                        vizkit_corridors.displayCorridor(corridor)
                    rescue Exception => e
                        puts "failed to display corridor #{corridor}: #{e}"
                    end
                end
            end
        else
            current_path = self.current_path
            controls.startIdx.setRange(0, current_path.size - 1)
            controls.endIdx.setRange(1, current_path.size)
            controls.startIdx.setValue(0)
            controls.endIdx.setValue(current_path.size)

            if @corridor_segments
                computeCorridorSegments
            else
                controls.endIdx.enabled = true
            end
            update_path
        end
    end

    def current_path
        all_paths[controls.pathIdx.value]
    end

    def setupUi
        @controls = Vizkit.load(File.join(File.dirname(__FILE__),'corridor_plan_view.ui'))

        layout = Qt::VBoxLayout.new(self)
        layout.add_widget(controls)

        @vizkit_corridors = view3d.createPlugin('corridor_planner')
        vizkit_corridors.setAlpha(0.5)
        vizkit_corridors.setZOffset(0.05)

        controls.pathIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            setPath(idx)
        end
        controls.startIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            controls.endIdx.setMinimum(idx + 1)
            update_path
        end
        controls.endIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            controls.startIdx.setMaximum(idx - 1)
            update_path
        end
        controls.btnExport.connect(SIGNAL('clicked()')) do
            export_current_corridor
        end

        controls.lstSymbol.connect(SIGNAL('activated(QString const&)')) do |value|
            if value == "None"
                controls.annotateSymbolIdx.enabled = false
                controls.btnAnnotateCorridor.enabled = false
                vizkit_corridors.setDisplayedAnnotation("")
            else
                controls.annotateSymbolIdx.enabled = true
                controls.btnAnnotateCorridor.enabled = true
                controls.btnSplit.enabled = true
                vizkit_corridors.setDisplayedAnnotation(value)
            end
        end

        controls.btnAnnotationOnPlan.connect(SIGNAL('clicked(bool)')) do |checked|
            vizkit_corridors.displayAnnotationsOnPlan(checked)
            update_path
        end

        controls.btnAnnotateCorridor.connect(SIGNAL('clicked()')) do
            puts controls.annotateSymbolIdx.value
            plan.annotate_corridor_segments(
                controls.lstSymbol.currentText, controls.annotateSymbolIdx.value,
                controls.lstSymbol.currentText)
            puts controls.annotateSymbolIdx.value
            vizkit_corridors.updatePlan(plan)
        end
        controls.btnSplit.connect(SIGNAL('clicked(bool)')) do |checked|
            if checked
                computeCorridorSegments
                update_path
            else
                controls.endIdx.enabled = true
                @corridor_segments = nil
                update_path
            end
        end
    end

    def computeCorridorSegments
        annotation_idx = plan.find_annotation(controls.lstSymbol.currentText)
        corridor = plan.path_to_corridor(current_path)
        @corridor_segments = corridor.split_annotation_segments(annotation_idx)
        controls.endIdx.enabled = false
        controls.startIdx.setValue(0)
        controls.startIdx.setMaximum(@corridor_segments.size)
    end

    def update_symbols
        lstSymbol = controls.lstSymbol

        if lstSymbol.count != 0
            currentSymbol = lstSymbol.currentText
        end

        lstSymbol.clear
        lstSymbol.addItem("None", Qt::Variant.new(-1))
        plan.annotation_symbols.each_with_index do |symbol_name, idx|
            lstSymbol.addItem(symbol_name, Qt::Variant.new(idx))
        end
        if currentSymbol
            idx = lstSymbol.findText(currentSymbol)
            if idx != -1
                lstSymbol.setCurrentIndex(idx)
            end
        end
    end

    def update_path
        if @plan && controls.pathIdx.value != -1
            begin
                if @corridor_segments
                    @current_corridor = @corridor_segments[controls.startIdx.value].last
                else
                    path = current_path[controls.startIdx.value, controls.endIdx.value - controls.startIdx.value]
                    @current_corridor = plan.path_to_corridor(path)
                end
                vizkit_corridors.clearCorridors(0)
                vizkit_corridors.displayCorridor(@current_corridor)
                controls.btnExport.enabled = true
            rescue Exception => e
                controls.btnExport.enabled = false
                STDERR.puts "ERROR: cannot display path #{path.inspect}"
                STDERR.puts "ERROR:   #{e.message}"
                e.backtrace.each do |line|
                    STDERR.puts "ERROR:     #{line}"
                end
            end
        else
            @current_corridor = nil
            controls.btnExport.enabled = false
        end
    end
end

