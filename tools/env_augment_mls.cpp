#include <envire/Core.hpp>
#include <envire/maps/GridBase.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/GridFloatToMLS.hpp>
#include <boost/scoped_ptr.hpp>

#include <iostream>

using namespace envire;

void usage(int exit_code = 0)
{
    std::cerr << "usage: env_augment_mls <env_path> <grid_map_id> <grid_band_name> <mls_map_id>\n"
	<< "   will augment an existing mls with color information from a classification band. "
        << std::endl;
    exit(exit_code);
}

int main(int argc, char* argv[])
{
    if (argc != 5)
    {
        std::cerr << "wrong number of arguments" << std::endl;
        usage(1);
    }

    std::string env_path(argv[1]);
    int grid_map_id = boost::lexical_cast<int>(argv[2]);
    std::string grid_band_name = argv[3];
    int mls_map_id  = boost::lexical_cast<int>(argv[4]);

    boost::scoped_ptr<envire::Environment> env(Environment::unserialize(env_path));

    envire::Grid<double>::Ptr input(env->getItem<envire::Grid<double> >(grid_map_id));
    envire::MLSGrid::Ptr  output(env->getItem<envire::MLSGrid>(mls_map_id));

    // make sure input and output have the same size
    assert( input->getWidth() == output->getWidth() );
    assert( input->getHeight() == output->getHeight() );

    // get max_force band
    envire::Grid<double>::ArrayType &max_force( input->getGridData( grid_band_name ) );

    // get the rgb image
    envire::ImageRGB24::Ptr image;
    std::vector<ImageRGB24*> images = env->getItems<envire::ImageRGB24>();
    if( images.empty() )
    {
	image = new envire::ImageRGB24( input->getWidth(), input->getHeight(), input->getScaleX(), input->getScaleY() );
	env->setFrameNode( image.get(), input->getFrameNode() );
    }
    else
    {
	image = images.front();
    }

    envire::ImageRGB24::ArrayType
	&r( image->getGridData( envire::ImageRGB24::R ) ),
	&g( image->getGridData( envire::ImageRGB24::G ) ),
	&b( image->getGridData( envire::ImageRGB24::B ) );

    for( size_t x = 0; x < input->getWidth(); x++ )
    {
	for( size_t y = 0; y < input->getHeight(); y++ )
	{
	    double force_value = max_force[y][x];

	    // converting grid values to rgb values in mls
	    base::Vector3d color = base::Vector3d::Zero();
	    if( force_value < 5.0 )
		color.setZero();
	    else if( force_value < 15.0 )
		color[0] = 1.0;
	    else if( force_value < 25.0 )
		color[2] = 1.0;
	    else if( force_value < 35.0 )
		color[1] = 1.0;

	    // colorize all the patches in a cell
	    for( envire::MLSGrid::iterator it = output->beginCell( x, y ); it != output->endCell(); it++ )
	    {
		it->color = color;
	    }

	    // set in the image
	    r[y][x] = color[0] * 255;
	    g[y][x] = color[1] * 255;
	    b[y][x] = color[2] * 255;
	}
    }

    // switch on color information in mls
    output->setHasCellColor( true );

    // and write to disk
    env->serialize(env_path);
    return 0;
}


