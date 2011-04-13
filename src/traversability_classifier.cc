#include "traversability_classifier.hh"

using namespace corridor_planner;
using envire::Grid;

ENVIRONMENT_ITEM_DEF( TraversabilityClassifier );

TraversabilityClassifier::TraversabilityClassifier(
        double weight_force,
        double force_threshold,
        double max_speed,
        int class_count,
        double min_width,
        double ground_clearance)
    : weight_force(weight_force)
    , force_threshold(force_threshold)
    , max_speed(max_speed)
    , class_count(class_count)
    , min_width(min_width)
    , ground_clearance(ground_clearance)
{
    for (int i = 0; i < INPUT_COUNT; ++i)
        input_layers_id[i] = -1;
}

TraversabilityClassifier::TraversabilityClassifier(envire::Serialization& so)
    : envire::Operator(so)
{
    unserialize(so);
}

void TraversabilityClassifier::serialize(envire::Serialization& so)
{
    Operator::serialize(so);

    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        if (input_layers_id[i] != -1 && !input_bands[i].empty())
        {
            so.write("input" + boost::lexical_cast<std::string>(i), input_layers_id[i]);
            so.write("input" + boost::lexical_cast<std::string>(i) + "_band", input_bands[i]);
        }
    }

    so.write("weight_force", weight_force);
    so.write("force_threshold", force_threshold);
    so.write("max_speed", max_speed);
    so.write("class_count", class_count);
    so.write("ground_clearance", ground_clearance);
    so.write("min_width", min_width);
    so.write("output_band", output_band);
}

void TraversabilityClassifier::unserialize(envire::Serialization& so)
{
    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        std::string input_key = "input" + boost::lexical_cast<std::string>(i);
        if (so.hasKey(input_key))
        {
            input_layers_id[i] = so.read<int>(input_key);
            input_bands[i] = so.read<std::string>(input_key + "_band");
        }
    }

    so.read<double>("weight_force", weight_force);
    so.read<double>("force_threshold", force_threshold);
    so.read<double>("max_speed", max_speed);
    so.read<int>("class_count", class_count);
    so.read<double>("ground_clearance", ground_clearance);
    so.read<double>("min_width", min_width);
    so.read<std::string>("output_band", output_band);
}

envire::Grid<double>* TraversabilityClassifier::getInputLayer(INPUT_DATA index) const
{
    if (input_layers_id[index] == -1)
        return 0;
    return getEnvironment()->getItem< Grid<double> >(input_layers_id[index]).get();
}
std::string TraversabilityClassifier::getInputBand(INPUT_DATA index) const
{ return input_bands[index]; }

envire::Grid<double>* TraversabilityClassifier::getSlopeLayer() const { return getInputLayer(SLOPE); }
std::string TraversabilityClassifier::getSlopeBand() const { return getInputBand(SLOPE); }
void TraversabilityClassifier::setSlope(Grid<double>* grid, std::string const& band_name)
{
    addInput(grid);
    input_layers_id[SLOPE] = grid->getUniqueId();
    input_bands[SLOPE] = band_name;

}

envire::Grid<double>* TraversabilityClassifier::getMaxStepLayer() const { return getInputLayer(MAX_STEP); }
std::string TraversabilityClassifier::getMaxStepBand() const { return getInputBand(MAX_STEP); }
void TraversabilityClassifier::setMaxStep(Grid<double>* grid, std::string const& band_name)
{
    addInput(grid);
    input_layers_id[MAX_STEP] = grid->getUniqueId();
    input_bands[MAX_STEP] = band_name;
}

envire::Grid<double>* TraversabilityClassifier::getMaxForceLayer() const { return getInputLayer(MAX_FORCE); }
std::string TraversabilityClassifier::getMaxForceBand() const { return getInputBand(MAX_FORCE); }
void TraversabilityClassifier::setMaxForce(Grid<double>* grid, std::string const& band_name)
{
    addInput(grid);
    input_layers_id[MAX_FORCE] = grid->getUniqueId();
    input_bands[MAX_FORCE] = band_name;
}

void TraversabilityClassifier::setOutput(OutputLayer* grid, std::string const& band_name)
{
    removeOutputs();
    addOutput(grid);
    output_band = band_name;
}

bool TraversabilityClassifier::updateAll()
{
    std::cout << "update: max_speed=" << max_speed << std::endl;
    OutputLayer* output_layer = getOutput< OutputLayer* >();
    if (!output_layer)
        throw std::runtime_error("TraversabilityClassifier: no output band set");

    OutputLayer::ArrayType& result = output_band.empty() ?
        output_layer->getGridData() :
        output_layer->getGridData(output_band);

    if (output_band.empty())
        output_layer->setNoData(CLASS_UNKNOWN);
    else
        output_layer->setNoData(output_band, CLASS_UNKNOWN);

    static double const DEFAULT_UNKNOWN_INPUT = -std::numeric_limits<double>::infinity();
    Grid<double> const* input_layers[3] = { 0, 0, 0 };
    double input_unknown[3];

    boost::multi_array<double, 2> const* inputs[3] = { 0, 0, 0 };
    bool has_data = false;
    for (int i = 0; i < INPUT_COUNT; ++i)
    {
        if (input_layers_id[i] != -1 && !input_bands[i].empty())
        {
            input_layers[i] = getEnvironment()->getItem< Grid<double> >(input_layers_id[i]).get();
            has_data = true;
            inputs[i] = &(input_layers[i]->getGridData(input_bands[i]));

            std::pair<double, bool> no_data = input_layers[i]->getNoData(input_bands[i]);
            if (no_data.second)
            {
                std::cout << "band " << i << " no_data=" << no_data.first << std::endl;
                input_unknown[i] = no_data.first;
            }
            else
                input_unknown[i] = DEFAULT_UNKNOWN_INPUT;
        }
    }
    if (!has_data)
        throw std::runtime_error("TraversabilityClassifier: no input layer configured");

    bool const has_slope = inputs[SLOPE],
         has_max_step = inputs[MAX_STEP],
         has_max_force = inputs[MAX_FORCE];
    if (has_max_step && ground_clearance == 0)
        throw std::runtime_error("a max_step band is available, but the ground clearance is set to zero");

    double const class_width = 1.0 / class_count;
                
    int width = output_layer->getWidth(), height = output_layer->getHeight();
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // Read the values for this cell. Set to CLASS_UNKNOWN and ignore the cell
            // if one of the available input bands has no information
            double values[INPUT_COUNT];
            int band_idx = 0;
            for (; band_idx < INPUT_COUNT; ++band_idx)
            {
                if (!inputs[band_idx]) continue;

                double value = (*inputs[band_idx])[y][x];
                if (value == input_unknown[band_idx])
                {
                    result[y][x] = CLASS_UNKNOWN;
                    break;
                }
                values[band_idx] = value;
            }
            if (band_idx != INPUT_COUNT) // found an unknown value
                continue;

            // First, max_step is an ON/OFF threshold on the ground clearance
            // parameter
            if (has_max_step && values[MAX_STEP] > ground_clearance)
            {
                result[y][x] = CLASS_OBSTACLE;
                continue;
            }

            // Compute an estimate of the force that the system can use to
            // propulse itself, using the max_force / slope values (and using
            // the maximum values if none are available)
            //
            // The result is then a linear mapping of F=[O, force_threshold] to
            // [0, 1]
            double max_force = force_threshold;
            double speed = 1;
            if (has_max_force)
                max_force = values[MAX_FORCE];
            if (has_slope)
                max_force = max_force - weight_force * fabs(sin(values[SLOPE]));

            if (max_force <= 0)
                result[y][x] = CLASS_OBSTACLE;
            else
            {
                if (max_force < force_threshold)
                    speed *= max_force / force_threshold;

                int klass = rint(speed / class_width);
                result[y][x] = CUSTOM_CLASSES + klass;
            }
        }
    }

    closeNarrowPassages(*output_layer, output_band, min_width);
    std::cout << "end update: max_speed=" << max_speed << std::endl;
    return true;
}

struct RadialLUT
{
    int centerx, centery;
    unsigned int width, height;
    boost::multi_array<std::pair<int, int>, 2>  parents;
    boost::multi_array<bool, 2> in_distance;

    void precompute(double distance, double scalex, double scaley)
    {
        double const radius2 = distance * distance;

        width  = 2* ceil(distance / scalex) + 1;
        height = 2* ceil(distance / scaley) + 1;
        in_distance.resize(boost::extents[height][width]);
        std::fill(in_distance.data(), in_distance.data() + in_distance.num_elements(), false);
        parents.resize(boost::extents[height][width]);
        std::fill(parents.data(), parents.data() + parents.num_elements(), std::make_pair(-1, -1));

        centerx = width  / 2;
        centery = height / 2;
        parents[centery][centerx] = std::make_pair(-1, -1);

        for (unsigned int y = 0; y < height; ++y)
        {
            for (unsigned int x = 0; x < width; ++x)
            {
                int dx = (centerx - x);
                int dy = (centery - y);
                if (dx == 0 && dy == 0) continue;

                double d2 = dx * dx * scalex * scalex + dy * dy * scaley * scaley;
                in_distance[y][x] = (d2 < radius2);
                if (abs(dx) > abs(dy))
                {
                    int parentx = x + dx / abs(dx);
                    int parenty = y + rint(static_cast<double>(dy) / abs(dx));
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
                else
                {
                    int parentx = x + rint(static_cast<double>(dx) / abs(dy));
                    int parenty = y + dy / abs(dy);
                    parents[y][x] = std::make_pair(parentx, parenty);
                }
            }
        }
    }

    void markAllRadius(boost::multi_array<uint8_t, 2>& result, int result_width, int result_height, int centerx, int centery, int value)
    {
        for (unsigned int y = 0; y < height; ++y)
        {
            int map_y = centery + y - this->centery;
            if (map_y < 0 || map_y >= result_height)
                continue;

            for (unsigned int x = 0; x < width; ++x)
            {
                int map_x = centerx + x - this->centerx;
                if (map_x < 0 || map_x >= result_width)
                    continue;
                if (result[map_y][map_x] != value)
                    continue;
                if (!in_distance[y][x])
                    continue;
                markSingleRadius(result, centerx, centery, x, y, value);
            }
        }
    }

    void markSingleRadius(boost::multi_array<uint8_t, 2>& result, int centerx, int centery, int x, int y, int value)
    {
        boost::tie(x, y) = parents[y][x];
        while (x != -1 && y != -1)
        {
            uint8_t& current = result[centery + y - this->centery][centerx + x - this->centerx];
            if (current == value)
                return;

            current = value;
            boost::tie(x, y) = parents[y][x];
        }
    }
};

void TraversabilityClassifier::closeNarrowPassages(TraversabilityClassifier::OutputLayer& map, std::string const& band_name, double min_width)
{
    RadialLUT lut;
    lut.precompute(min_width, map.getScaleX(), map.getScaleY());
    for (unsigned int y = 0; y < lut.height; ++y)
    {
        for (unsigned int x = 0; x < lut.width; ++x)
            std::cout << "(" << lut.parents[y][x].first << " " << lut.parents[y][x].second << ") ";
        std::cout << std::endl;
    }
    std::cout << std::endl;
    for (unsigned int y = 0; y < lut.height; ++y)
    {
        for (unsigned int x = 0; x < lut.width; ++x)
            std::cout << "(" << lut.in_distance[y][x] << " ";
        std::cout << std::endl;
    }

    boost::multi_array<uint8_t, 2>& data = map.getGridData(band_name);
    for (unsigned int y = 0; y < map.getHeight(); ++y)
    {
        for (unsigned int x = 0; x < map.getWidth(); ++x)
        {
            int value = data[y][x];
            if (value == CLASS_OBSTACLE)
                lut.markAllRadius(data, map.getWidth(), map.getHeight(), x, y, CLASS_OBSTACLE);
        }
    }
}

