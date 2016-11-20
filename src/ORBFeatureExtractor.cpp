//
// ORBFeatureExtractor.cpp
//
// Extract ORB feature of each frame and feed 
// the extracted feature descriptor and position
// into the data manager.
//
// @Yu

#include "Common.h"
#include "ORBFeatureExtractor.h"

void ORBFeatureExtractor::process(DataManager& data, Frame& frame) {

	RawBuffer frameBuffer = frame.frameBuffer;
	std::vector<cv::KeyPoint> keypoints;
	detector.detect(frameBuffer, keypoints);
	Mat descriptor;
	extractor.compute(frameBuffer, keypoints, descriptor);
	for (int i=0; i<keypoints.size(); i++)
	{
		Feature feature;
		feature.position = keypoints[i].pt;
		feature.descriptor = descriptor.row(i);
	}

	Mat visualization;
	drawKeypoints(frameBuffer, keypoints,visualization, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	imshow("Keypoint", visualization);
	waitKey(40);
}

bool ORBFeatureExtractor::validationCheck(DataManager& data, Frame& frame) {
	return true;
}

