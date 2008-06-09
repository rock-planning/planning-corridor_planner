#ifndef __SKELETON_HH__
#define __SKELETON_HH__

#include <vector>
#include <stdint.h>
#include <opencv/cv.h>
#include "point.hh"

namespace Nav
{
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

        void initializeHeightMap(size_t max);
        void propagateHeightMap(uint8_t** free_cList);
        std::vector<int> hillClimbing();
        
    public:
        SkeletonExtraction(size_t width, size_t height);
        ~SkeletonExtraction();

        /** Extracts the skeleton of the given BGR image */
        std::vector<int> processColorImage(IplImage* bgraImage);

        /** Extracts the skeleton of the given grayscale image */
        std::vector<int> processGrayImage(IplImage* bgraImage);

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        std::vector<int> processEdgeImage(uint8_t const* edge_image, int threshold = 127);

        /** Extracts the skeleton from the given edge image (i.e. greyscale image
         * of +width x height+ where white is an edge and black a valley */
        std::vector<int> processEdgeSet(PointSet const& edges);
    };
}

#endif

