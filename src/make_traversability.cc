#include "envire/Core.hpp"
#include "envire/maps/Grid.hpp"
#include "envire/maps/MLSGrid.hpp"
#include "traversability_classifier.hh"

#include "boost/scoped_ptr.hpp"

using namespace envire;
using namespace corridor_planner;
using namespace std;
     
int main( int argc, char* argv[] )
{
    if( argc < 2 || argc > 7)
    {
	std::cout << "usage: make_traversability input mass force_threshold max_speed [class_count [ground_clearance]]" << std::endl;
	std::cout << "usage: make_traversability input" << std::endl;
        std::cout << "  generates a classified map of traversabilities based on the given map (a single map must contain all the necessary bands)" << std::endl;
        std::cout << "  the generated map will have the same width, height and cell size than the source map" << std::endl;
        std::cout << std::endl;
        std::cout << "  If the parameters are given, a new operator and map are created" << std::endl;
        std::cout << "  If only the input environment is provided, then the system will reuse an existing traversability classification operator" << std::endl;
	exit(0);
    }
    boost::scoped_ptr<Environment> env(Environment::unserialize( argv[1] ));

    std::string slope_band = "mean_slope";
    std::string step_band  = "corrected_max_step";
    std::string force_band = "max_force";

    // Check if there is already an output map and operator that match what is
    // asked
    boost::intrusive_ptr< Grid<double> > input = env->getItem< Grid<double> >();

    if (argc == 2)
    {
        std::vector<TraversabilityClassifier*> operators =
            env->getItems<TraversabilityClassifier>();

        vector<TraversabilityClassifier*>::iterator it = operators.begin();
        for (; it != operators.end(); ++it)
        {
            TraversabilityClassifier const& op = **it;
            if (op.getMaxStepLayer() && (op.getMaxStepLayer() != input || op.getMaxStepBand() != step_band))
                continue;
            if (op.getSlopeLayer() && (op.getSlopeLayer() != input || op.getSlopeBand() != slope_band))
                continue;
            if (op.getMaxForceLayer() && (op.getMaxForceLayer() != input || op.getMaxForceBand() != force_band))
                continue;

            break;
        }

        if (it != operators.end())
        {
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "Reusing existing traversability classification operator" << std::endl;
            env->updateOperators();
            env->serialize(argv[1]);
            exit(0);
        }
        else
        {
            std::cout << "There are no existing traversability classification operator in this environment" << std::endl;
            exit(1);
        }
    }

    env->updateOperators();

    boost::intrusive_ptr< TraversabilityClassifier::OutputLayer > output = new TraversabilityClassifier::OutputLayer(
            input->getWidth(), input->getHeight(),
            input->getScaleX(), input->getScaleY());
    env->attachItem(output.get());
    output->setFrameNode(input->getFrameNode());

    double weight_force = boost::lexical_cast<double>(argv[2]) * 9.81;
    double force_threshold = boost::lexical_cast<double>(argv[3]);
    double max_speed = boost::lexical_cast<double>(argv[4]);
    int class_count = 10;
    if (argc > 5)
        class_count = boost::lexical_cast<int>(argv[5]);

    double min_width = 0.5;
    if (argc > 6)
        min_width = boost::lexical_cast<int>(argv[6]);

    double ground_clearance = 0;
    if (argc > 7)
        ground_clearance = boost::lexical_cast<int>(argv[7]);

    // Create the convertion operator and run it
    corridor_planner::TraversabilityClassifier *op = new TraversabilityClassifier(
            weight_force, force_threshold, max_speed, class_count, min_width, ground_clearance);
    env->attachItem( op );
    if (input->hasBand("mean_slope"))
        op->setSlope(input.get(), "mean_slope");
    if (input->hasBand("corrected_max_step"))
        op->setMaxStep(input.get(), "corrected_max_step");
    if (input->hasBand("max_force"))
        op->setMaxForce(input.get(), "max_force");
    op->setOutput(output.get(), "traversability_class");
    op->updateAll();

    // detach the resulting pointcloud from the existing environment, and place
    // into a newly created one.
    env->serialize(argv[1]);
} 

