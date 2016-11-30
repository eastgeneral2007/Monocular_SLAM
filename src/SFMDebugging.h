//
// SFMDebugging.h
//
// a bunch of GUI debugging tools for Structure 
// from Motion.
//
// @Yu

#include "CommonCV.h"

/**
 * drawEpipolarLine
 * 
 * The usr click a point on the right image, the corresponding 
 * epipolar line in the right image will be drawn.
 */
void drawEpipolarLine(const Mat& F, const Mat& img1, const Mat& img2);

/**
 * visualizeFeatureMatching
 *
 * visualize the feature matching correspondence
 * of two frames.
 */
 void visualizeFeatureMatching(const Mat& img1, const Mat& img2, 
 							   const vector<Point2d>& pos1, const vector<Point2d>& pos2, 
							   const vector<DMatch>& matches);

/**
 * check whether a rotation matrix is valid (determinant =1)
 */
bool CheckValidRotation(const Mat& rotation);

/**
 * check whether an essential matrix is valid (whether it is singular matrix)
 */
bool CheckValidEssential(const Mat& essential);