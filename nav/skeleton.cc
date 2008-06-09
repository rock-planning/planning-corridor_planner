#include "skeleton.hh"
#include <stdint.h>
#include <iostream>

using std::vector;
using namespace Nav;

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

void SkeletonExtraction::initializeHeightMap(size_t maxDist)
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

}

void SkeletonExtraction::propagateHeightMap(uint8_t** free_cList)
{
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
}

PointSet SkeletonExtraction::hillClimbing()
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

    PointSet result;
    BPListItem* freeBorderPoint = &borderPoints[0];
    HcMem* hill_item = &hcsList[0];
    while ((*hill_item).actPos) {
        uint8_t* height_item = (*hill_item).actPos;
        uint32_t height_value = *height_item;
        for (int i = 0; i < 8; ++i){
            uint8_t* neighbour = height_item + displacement[i];
            uint32_t neighbour_value = *neighbour;
            if (neighbour_value > height_value){
                height_value = neighbour_value;
                (*hill_item).actPos = neighbour;
            }
        }

        if (height_item == hill_item->actPos){
            int point_index = height_item - &heightmap[0];
            int x = point_index % width;
            int y = point_index / width;
            result.insert( PointID(x, y) );

            BPListItem** actSkelPoint = &skelPoints[point_index];
            (*freeBorderPoint).borderPoint = (*hill_item).origin;
            if (*actSkelPoint){
                (*freeBorderPoint).nextItem    = *actSkelPoint;
            } else {
                (*freeBorderPoint).nextItem    = NULL;
            }

            *actSkelPoint = freeBorderPoint;
            freeBorderPoint++;

            free_item--;
            *hill_item = *free_item;
            (*free_item).actPos = 0;
        }
        hill_item++;
        if (!hill_item->actPos) hill_item = &hcsList[0];
    }

    return result;
}

PointSet SkeletonExtraction::processGrayImage(IplImage* bgraImage)
{
    cvCanny(grayImg,edgeImg,70, 210, 3);
    return processEdgeImage(reinterpret_cast<uint8_t*>(edgeImg->imageData));
}

PointSet SkeletonExtraction::processColorImage(IplImage* bgraImage)
{
    cvCvtColor(bgraImage,grayImg,CV_BGRA2GRAY);
    return processGrayImage(grayImg);
}

PointSet SkeletonExtraction::processEdgeImage(uint8_t const* edges, int threshold)
{
    initializeHeightMap(128);

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

    propagateHeightMap(free_cList);
    return hillClimbing();
}

PointSet SkeletonExtraction::processEdgeSet(PointSet const& edges)
{
    initializeHeightMap(128);

    uint8_t ** free_cList = &cList[0]; 
    for (PointSet::const_iterator it = edges.begin(); it != edges.end(); ++it)
    {
        if (it->x == 0 || it->x >= width - 1 ||
                it->y == 0 || it->y >= height - 1)
            continue;

        *free_cList = &heightmap[it->y * width + it->x];
        **free_cList = 0;
        ++free_cList;
    }

    propagateHeightMap(free_cList);
    return hillClimbing();
}

