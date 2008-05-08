$LOAD_PATH.unshift File.expand_path(File.dirname(__FILE__))
require 'plot.rb'

class Plotter
    def run
        t.save_dir = '.'

        # First, extract the blocks
        io     = StringIO.new(@data)
        blocks = [String.new]
        io.each_line do |line|
            if line == "\n"
                blocks << String.new
            else
                blocks.last << line
            end
        end

        puts "found #{blocks.size} blocks"

        # Then, generate a graph figure and append a sequence number to it
        blocks.each_with_index do |block, i|
            next if block.empty?
            @data = block
            t.make_pdf('Graph')
            if source = t.figure_pdf('Graph')
                target = "%s-%05i.pdf" % [source[0..-5], i]
                system 'mv', source, target
            else
                puts "no figure for #{i}: empty plot ?"
            end
        end
    end
end

Plotter.new(STDIN).run

