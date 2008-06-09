#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include <vector>
#include <stdint.h>
#include <opencv/cv.h>

class SkeletonExtraction
{
    struct HcMem {
        uint8_t* actPos;
        uint8_t* origin;
    };

    struct BPListItem {
        uint8_t*    borderPoint;
        BPListItem* nextItem;
    };

    struct SPListItem {
        BPListItem** skelPoint;
        bool            newList;
    };

    size_t width, height;

    // Canny stuff
    IplImage* grayImg;
    IplImage* edgeImg;

    std::vector<uint32_t>    edges;
    std::vector<uint8_t>     heightmap;
    std::vector<uint8_t*>    cList;
    std::vector<HcMem>       hcsList;
    std::vector<BPListItem>  borderPoints;
    std::vector<BPListItem*> skelPoints;
    std::vector<SPListItem>  skelPointList;
    
public:
    SkeletonExtraction(size_t width, size_t height);
    ~SkeletonExtraction();

    /** Extracts the skeleton of the given BGR image */
    std::vector<int> extractColor(IplImage* bgraImage);

    /** Extracts the skeleton of the given grayscale image */
    std::vector<int> extractGray(IplImage* bgraImage);

    /** Extracts the skeleton from the given edge image (i.e. greyscale image
     * of +width x height+ where white is an edge and black a valley */
    std::vector<int> extract(uint8_t const* edge_image);

    void buildHeightMap(uint8_t const* edges, int threshold, int maxDist);
    std::vector<int> hillClimbing();
};

#endif

