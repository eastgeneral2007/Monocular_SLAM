#include "InitialCameraMotionEstimator.h"


void InitialCameraMotionEstimator::process(DataManager& data, int frameIdx)
{
	if (frameIdx == 0) {
		// TODO
		return;
	}

	Frame& curFrame = data.frames[frameIdx];
	Frame& preFrame = data.frames[frameIdx-1];

	BFMatcher matcher(NORM_HAMMING);
	vector<DMatch> matches;
	Mat descriptors1 = preFrame.features.descriptors;
	Mat descriptors2 = curFrame.features.descriptors;
	matcher.match(descriptors1, descriptors2, matches);

	// visualize the matches
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<Point2f>& positions1 = preFrame.features.positions;
	for (int i=0; i< positions1.size(); i++) {
		keypoints1.push_back(KeyPoint(positions1[i], 5));
	}
	vector<Point2f>& positions2 = curFrame.features.positions;
	for (int i=0; i< positions2.size(); i++) {
		keypoints2.push_back(KeyPoint(positions2[i], 5));
	}
	Mat visualization;
	drawMatches(preFrame.frameBuffer, keypoints1, curFrame.frameBuffer, keypoints2, matches, visualization);
	imshow("Matches", visualization);
	waitKey(40);

	// compute fundamental matrix and essential matrix
	Mat &K = data.camera_intrinsics;
	Mat F = findFundamentalMat(preFrame.features.positions, 
							   curFrame.features.positions, 
							   FM_RANSAC, 3., 0.99);

	//TODO: we need to load the camera intrinsics!
	//Mat_<double> E = K.t() * F * K;


}

bool InitialCameraMotionEstimator::validationCheck(DataManager& data, int frameIdx)
{
	return true;
}