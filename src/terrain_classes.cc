#include "terrain_classes.hh"
#include <fstream>
#include <sstream>

using namespace std;
using namespace corridor_planner;

std::list<TerrainClass> TerrainClass::load(std::string const& path)
{
    list<TerrainClass> classes;
    ifstream class_file(path.c_str());
    while (!class_file.eof())
    {
        char line[256];
        class_file.getline(line, 256);

        if (line[0] == '#' || !*line)
            continue;

        istringstream class_desc(line);
        TerrainClass klass;
        class_desc >> klass.in >> klass.out >> klass.cost >> klass.margin >> klass.name;
        classes.push_back(klass);
    }

    return classes;
}

