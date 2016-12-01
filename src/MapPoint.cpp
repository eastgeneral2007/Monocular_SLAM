//
// Created by JimXing on 12/11/16.
//

#include "MapPoint.h"

void MapPoint::addObservingFrame(Frame* frame, int featureIdx)
{
    assert(featureIdx >= 0);
    observerToIndex[frame] = featureIdx;
}

int MapPoint::getFeatureIdxFromObservingFrame(Frame* frame)
{
    if (observerToIndex.find(frame) == observerToIndex.end()) {
        return -1;
    }
    return observerToIndex[frame];
}

void MapPoint::deleteObservingFrame(Frame* frame)
{
    if (observerToIndex.find(frame) == observerToIndex.end()) {
        return;
    }
    observerToIndex.erase(frame);
}
