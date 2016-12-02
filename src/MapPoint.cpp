//
// Created by JimXing on 12/11/16.
//

#include "MapPoint.h"
#include "Frame.h"

void MapPoint::addObservingFrame(int frameIdx, int featureIdx)
{
    assert(featureIdx >= 0);
    observerToIndex[frameIdx] = featureIdx;
}

int MapPoint::getFeatureIdxFromObservingFrame(int frameIdx)
{
    if (observerToIndex.find(frameIdx) == observerToIndex.end()) {
        return -1;
    }
    return observerToIndex[frameIdx];
}

void MapPoint::deleteObservingFrame(int frameIdx)
{
    if (observerToIndex.find(frameIdx) == observerToIndex.end()) {
        return;
    }
    observerToIndex.erase(frameIdx);
}
