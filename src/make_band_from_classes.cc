#include "envire/Core.hpp"
#include "envire/maps/Grid.hpp"
// #include "envire/maps/MLSGrid.hpp"
// #include "traversability_classifier.hh"

// #include "boost/scoped_ptr.hpp"

using namespace envire;
// using namespace corridor_planner;
using namespace std;

static double const UNKNOWN = -std::numeric_limits<double>::infinity();
struct PixelClass
{
    int red, green, blue;
    double value;
    std::string description;

    PixelClass()
        : value(UNKNOWN) {}
};
     
int main( int argc, char* argv[] )
{
    if( argc != 5 )
    {
	std::cout << "usage: make_band_from_classes input_map target_map target_band classes_file" << std::endl;
        std::cout << "  generates a map of floating-point values based on a RGB file and a file that describe a mapping from color to value" << std::endl;
        std::cout << "  the generated map will have the same width, height and cell size than the source map" << std::endl;
	exit(0);
    }
    std::string input_path(argv[1]);
    std::string target_path(argv[2]);
    std::string target_band(argv[3]);
    std::string classes_path(argv[4]);

    // Load the input map
    boost::intrusive_ptr< Grid<uint8_t> > class_map = new Grid<uint8_t>();
    std::vector<std::string> bands;
    bands.push_back("red");
    bands.push_back("green");
    bands.push_back("blue");
    class_map->readGridData(bands, input_path);
    boost::multi_array<uint8_t, 2> class_red_band   = class_map->getGridData("red");
    boost::multi_array<uint8_t, 2> class_green_band = class_map->getGridData("green");
    boost::multi_array<uint8_t, 2> class_blue_band  = class_map->getGridData("blue");

    // And create the output map
    boost::intrusive_ptr< Grid<double> > output_map = new Grid<double>(class_map->getWidth(), class_map->getHeight(), class_map->getScaleX(), class_map->getScaleY());
    boost::multi_array<double, 2>& output_band = output_map->getGridData(target_band);
    output_map->setNoData(UNKNOWN);

    // Generate the mapping from (r, g, b) to values
    boost::multi_array<double, 3> classes;
    classes.resize( boost::extents[256][256][256] );
    fill(classes.data(), classes.data() + classes.num_elements(), UNKNOWN);
    ifstream class_file(classes_path.c_str());
    while (!class_file.eof())
    {
        char line[256];
        class_file.getline(line, 256);
        if (line[0] == '#' || !*line) continue;
        istringstream class_desc(line);
        PixelClass klass;
        class_desc >> klass.red >> klass.green >> klass.blue >> klass.value >> klass.description;
        classes[klass.red][klass.green][klass.blue] = klass.value;
    }

    // Do the convertion
    uint8_t* class_red = class_red_band.data();
    uint8_t* class_green = class_green_band.data();
    uint8_t* class_blue = class_blue_band.data();
    double*  output = output_band.data();
    for (unsigned int i = 0; i < output_band.num_elements(); ++i)
    {
        int red   = class_red[i];
        int green = class_green[i];
        int blue  = class_blue[i];
        output[i] = classes[red][green][blue];
    }

    // And save the result
    output_map->writeGridData(target_band, target_path);
} 

