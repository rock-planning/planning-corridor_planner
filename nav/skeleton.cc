#include "skeleton.hh"
#include <stdint.h>
#include <iostream>

using std::vector;

SkeletonExtraction::SkeletonExtraction(size_t width, size_t height)
    : width(width)
    , height(height)
    , edges(width * height)
    , heightmap(width * height)
    , cList(width * height)
    , hcsList(width * height)
    , borderPoints(width * height)
    , skelPoints(width * height)
    , skelPointList(width * height)
{
    // Canny stuff
    grayImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    edgeImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
}

SkeletonExtraction::~SkeletonExtraction()
{
    cvReleaseImage(&grayImg);
    cvReleaseImage(&edgeImg);
}

void SkeletonExtraction::buildHeightMap(uint8_t const* edges, int threshold, int maxDist)
{
    /* All of the heightmap is set to +maxDist+, except the border
     * which is set to 0
     */
    memset(&heightmap[width], maxDist, heightmap.size() - 2 * width);
    memset(&heightmap[0],     0, width);
    memset(&heightmap[width * (height - 1)], 0, width);
    for (size_t y = 1; y < height-1; ++y) {
        heightmap[width * y] = 0;
        heightmap[width * y + width - 1] = 0;
    }

    /* The set of points of +heightmap+ whose value in +edges+ is more than
     * +threshold+ is saved into cList. In the meantime, their heightmap value
     * is set to 0.
     */
    // free_cList ist always first free entry in cList
    uint8_t ** free_cList = &cList[0]; 
    {
        uint8_t const *aem = &edges[width + 1];
        uint8_t *ahm = &heightmap[width + 1];
        for (size_t y = 1; y < height-1; ++y){
            for (size_t x = 1; x < width-1; ++x){
                if (*aem > threshold){
                    *free_cList = ahm;
                    *ahm = 0;
                    free_cList++;
                }
                aem++;
                ahm++;
            }
            aem += 2;
            ahm += 2;
        }
    }

    const int32_t addVal[8] = { 1, 2, 2, 1, 2, 2, 1, 1 };
    const int32_t displacement[8] = {
        -width, -width - 1, -width + 1, width,
        width - 1, width + 1, -1, +1 };

    uint8_t **candidate_ptr = &cList[0];
    while (cList[0]) {
        uint8_t *candidate       = *candidate_ptr;
        uint32_t candidate_value = *candidate;
        for (size_t i = 0; i < 8; ++i){
            uint8_t* neighbour        = candidate + displacement[i];
            uint32_t propagated_value = candidate_value + addVal[i]; 
            if (*neighbour > propagated_value){
                *free_cList = neighbour; free_cList++;
                *neighbour  = propagated_value;
            }
        }

        free_cList--;
        *candidate_ptr = *free_cList;
        *free_cList = 0;
        candidate_ptr++;

        if (!*candidate_ptr) 
            candidate_ptr = &cList[0];
    }

    for (size_t y = 0; y < height; ++y)
    {
        for (size_t x = 0; x < width; ++x)
            std::cout << (int) heightmap[y * width + x] << " ";
        std::cout << "\n";
    }
}

std::vector<int> SkeletonExtraction::hillClimbing()
{
    memset(&hcsList[0],   0, sizeof(BPListItem*)*width*height);
    HcMem* free_item = &hcsList[0];
    uint8_t* height_item = &heightmap[width + 1];
    for (size_t y = 1; y < height - 1; ++y) {
        for (size_t x = 1; x < width - 1; ++x) {
            if (*height_item == 1) {
                (*free_item).actPos = height_item;
                (*free_item).origin = height_item;
                free_item++;
            }
            height_item++;
        }
        height_item += 2;
    }

    memset(&skelPoints[0],   0, sizeof(BPListItem*)*width*height);
    memset(&borderPoints[0], 0, sizeof(BPListItem)*width*height);

    const int32_t displacement[8] = {
        -width, -width - 1, -width + 1, width,
        width - 1, width + 1, -1, +1 };

    BPListItem* freeBorderPoint = &borderPoints[0];
    HcMem* hill_item = &hcsList[0];
    vector<int> result;
    while ((*hill_item).actPos) {
        uint8_t* height_item = (*hill_item).actPos;
        uint32_t height_value = *height_item;
        bool progressed = false;
        for (int i = 0; i < 8; ++i){
            uint8_t* neighbour = height_item + displacement[i];
            uint32_t neighbour_value = *neighbour;
            if (neighbour_value > height_value){
                height_value = neighbour_value;
                (*hill_item).actPos = neighbour;
                progressed = true;
            }
        }
        if (!progressed){
            BPListItem** actSkelPoint = &skelPoints[0] + (height_item - &heightmap[0]);
            (*freeBorderPoint).borderPoint = (*hill_item).origin;
            if (*actSkelPoint){
                (*freeBorderPoint).nextItem    = *actSkelPoint;
            } else {
                (*freeBorderPoint).nextItem    = NULL;
            }

            *actSkelPoint = freeBorderPoint;
            freeBorderPoint++;
            *hill_item = *free_item;
            (*hill_item).actPos = 0;
            free_item--;
        }
        hill_item++;
        if (!hill_item->actPos) hill_item = &hcsList[0];
    }

    for (size_t y = 2; y < height - 2; ++y)
        for (size_t x = 2; x < width - 2; ++x)
        {
            size_t id = y * width + x;
            if (skelPoints[id])
                result.push_back(id);
        }

    return result;
}

vector<int> SkeletonExtraction::extractGray(IplImage* bgraImage)
{
    cvCanny(grayImg,edgeImg,70, 210, 3);
    return extract(reinterpret_cast<uint8_t*>(edgeImg->imageData));
}

vector<int> SkeletonExtraction::extractColor(IplImage* bgraImage)
{
    cvCvtColor(bgraImage,grayImg,CV_BGRA2GRAY);
    return extractGray(grayImg);
}

vector<int> SkeletonExtraction::extract(uint8_t const* edge_image)
{
    buildHeightMap(edge_image, 128, 127);
    return hillClimbing();
}

