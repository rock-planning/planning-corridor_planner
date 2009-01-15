#include <gdal.h>
#include <gdal_priv.h>

#include <iostream>
#include <algorithm>

#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
using namespace std;
using namespace boost;

struct TerrainClass
{
    int in;
    int out;
    float cost;
    string name;
};

static int distToClass(TerrainClass const& a, TerrainClass const& b, int ref)
{
    return abs(a.in - ref) < abs(b.in - ref);
}

int main(int argc, char** argv)
{
    GDALAllRegister();
    if (argc != 4)
    {
        cerr << "usage: fix_terrain_classes terrain_file classes_file output_file" << endl;
        return 1;
    }

    char const* filename = argv[1];
    GDALDataset* set = ((GDALDataset*) GDALOpen(filename, GA_ReadOnly));
    GDALRasterBand* band = set->GetRasterBand(1);
    if (band->GetRasterDataType() != GDT_Byte)
    {
        cerr << "wrong format" << endl;
        return 1;
    }

    int xSize = set->GetRasterXSize();
    int ySize = set->GetRasterYSize();

    vector<uint8_t> buffer;
    buffer.resize(xSize * ySize);
    band->RasterIO(GF_Read, 0, 0, xSize, ySize, &buffer[0], xSize, ySize, GDT_Byte, 0, 0);

    int klass_conv[256];
    for (int i = 0; i < 256; ++i)
        klass_conv[i] = -1;
    klass_conv[255] = 0;

    list<TerrainClass> classes;
    ifstream class_file(argv[2]);
    while (!class_file.eof())
    {
        char line[256];
        class_file.getline(line, 256);

        if (line[0] == '#' || !*line)
            continue;

        istringstream class_desc(line);
        TerrainClass klass;
        class_desc >> klass.in >> klass.out >> klass.cost >> klass.name;
        classes.push_back(klass);
        cerr << klass.name << ": " << klass.in << " => " << klass.out << "(" << klass.cost << ")\n";
        klass_conv[klass.in] = klass.in;
    }

    for (int i = 0; i < xSize * ySize; ++i)
    {
        int value = buffer[i];
        if (klass_conv[value] == -1)
        {
            list<TerrainClass>::const_iterator nearest =
                min_element(classes.begin(), classes.end(),
                        bind(&distToClass, _1, _2, value));
            klass_conv[value] = nearest->in;
        }

        buffer[i] = klass_conv[value];
    }
    delete set;

    // Now output the updated image
    string output_path = string(argv[3]);
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("Gtiff");
    auto_ptr<GDALDataset> output_set(driver->Create(output_path.c_str(), xSize, ySize, 1, GDT_Byte, 0));
    output_set->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, xSize, ySize, &buffer[0], xSize, ySize, GDT_Byte, 0, 0);
}

