#ifndef CORRIDOR_PLANNER_TRAVERSABILITY_CLASSIFIER_HH
#define CORRIDOR_PLANNER_TRAVERSABILITY_CLASSIFIER_HH

#include <envire/Core.hpp>
#include <envire/maps/Grid.hpp>

namespace corridor_planner {
    /** Classification of terrain into symbolic traversability classes, based on
     * different modalities.
     *
     * For now, the modalities that are used are:
     *
     * <ul>
     * <li>terrain's maximum allowed torque
     * <li>local slope
     * <li>maximum step size
     * </ul>
     *
     * It outputs a map in which each cell has an integer value, this integer
     * value being the traversability class for the cell
     *
     * If one of the modality is missing, it is simply ignored
     */
    class TraversabilityClassifier : public envire::Operator {
        ENVIRONMENT_ITEM( TraversabilityClassifier );

        enum INPUT_DATA {
            SLOPE,
            MAX_STEP,
            MAX_FORCE,
            INPUT_COUNT
        };

        enum CLASSES {
            CLASS_UNKNOWN = 0,
            CLASS_OBSTACLE = 1,
            CUSTOM_CLASSES = 2
        };

        /** Our input layers. This is stored here in addition to store it in the
         * operator graph, as we might use multiple bands of the same layer
         *
         * They are store as integers to support deserialization properly (we
         * have no way in unserialize() to get hold on our input layers)
         */
        int input_layers_id[INPUT_COUNT];
        /** The bands that should be used in the input layers */
        std::string input_bands[INPUT_COUNT];

        envire::Grid<double>* getInputLayer(INPUT_DATA index) const;
        std::string getInputBand(INPUT_DATA index) const;

        // Note: no need to store the output layer as it is accessible from the
        // operation graph
        std::string output_band;

        double weight_force;
        double force_threshold;
        double max_speed;
        int class_count;
        double min_width;
        double ground_clearance;

    public:
        typedef envire::Grid<uint8_t> OutputLayer;

        TraversabilityClassifier(
                double weight_force,
                double force_threshold,
                double max_speed,
                int class_count,
                double min_width,
                double ground_clearance);

        TraversabilityClassifier(envire::Serialization& so);

        envire::Grid<double>* getSlopeLayer() const;
        std::string getSlopeBand() const;
        void setSlope(envire::Grid<double>* grid, std::string const& band_name);

        envire::Grid<double>* getMaxStepLayer() const;
        std::string getMaxStepBand() const;
        void setMaxStep(envire::Grid<double>* grid, std::string const& band_name);

        envire::Grid<double>* getMaxForceLayer() const;
        std::string getMaxForceBand() const;
        void setMaxForce(envire::Grid<double>* grid, std::string const& band_name);

        std::string getOutputBand() const;
        void setOutput(OutputLayer* grid, std::string const& band_name);

        bool updateAll();
        void closeNarrowPassages(OutputLayer& map, std::string const& band_name, double min_width);

        void serialize(envire::Serialization& so);
        void unserialize(envire::Serialization& so);
    };
}

#endif

