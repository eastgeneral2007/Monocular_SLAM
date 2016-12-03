//
// CameraPoseEstimator.cpp
//
// This stage performs camera motion estimation
// for each frame based on the feature matching 
// with the previous frame. An initial camera
// matrix, which transforms from word space to
// camera space, is estimated. 
//
// @Yu

// #define DEBUG_CameraPoseEstimator

#include "CameraPoseEstimator.h"
#include "CommonMath.h"
#include "ParamConfig.h"
#include "SFMDebugging.h"

/**
 * triangulate a single point from two view
 * if the 3d point acquired from triangulation 
 * is in front of the both camera, return true, 
 * else return false.
 */
static bool TriangulateSinglePointFromTwoView(const Point2d& pts1, const Point2d& pts2,  
											  const Mat& Rt1, const Mat& Rt2, const Mat& K,
											  Point3d& result, bool countFront = false)
{
	// compute camera matrix
	Mat P1 = K * Rt1; Mat P2 = K * Rt2;

	// build the linear system
	Mat A;
	A.create(4, 4, CV_64F);
	A.at<double>(0,0) = P1.at<double>(0,0)-P1.at<double>(2,0)*pts1.x;
	A.at<double>(0,1) = P1.at<double>(0,1)-P1.at<double>(2,1)*pts1.x;
	A.at<double>(0,2) = P1.at<double>(0,2)-P1.at<double>(2,2)*pts1.x;
	A.at<double>(0,3) = P1.at<double>(0,3)-P1.at<double>(2,3)*pts1.x;
	A.at<double>(1,0) = P1.at<double>(1,0)-P1.at<double>(2,0)*pts1.y;
	A.at<double>(1,1) = P1.at<double>(1,1)-P1.at<double>(2,1)*pts1.y;
	A.at<double>(1,2) = P1.at<double>(1,2)-P1.at<double>(2,2)*pts1.y;
	A.at<double>(1,3) = P1.at<double>(1,3)-P1.at<double>(2,3)*pts1.y;
	A.at<double>(2,0) = P2.at<double>(0,0)-P2.at<double>(2,0)*pts2.x;
	A.at<double>(2,1) = P2.at<double>(0,1)-P2.at<double>(2,1)*pts2.x;
	A.at<double>(2,2) = P2.at<double>(0,2)-P2.at<double>(2,2)*pts2.x;
	A.at<double>(2,3) = P2.at<double>(0,3)-P2.at<double>(2,3)*pts2.x;
	A.at<double>(3,0) = P2.at<double>(1,0)-P2.at<double>(2,0)*pts2.y;
	A.at<double>(3,1) = P2.at<double>(1,1)-P2.at<double>(2,1)*pts2.y;
	A.at<double>(3,2) = P2.at<double>(1,2)-P2.at<double>(2,2)*pts2.y;
	A.at<double>(3,3) = P2.at<double>(1,3)-P2.at<double>(2,3)*pts2.y;
	
	// solve it
	Mat X; solveHLS(A, X);
	dehomogenize(X);
	Mat2Point3d(X, result);

	// check whether it is in front of the both camera
	if (countFront) {
		Mat pc1 = Rt1*X.t();
		Mat pc2 = Rt2*X.t();
		if (pc1.at<double>(2)>0 && pc2.at<double>(2)>0) return true;
		else return false;
	}
	return true;
}


/**
 * Perform triangulation, given the camera matrix and keypoints correspondences.
 * We will perform triangulation by minimizing the projection error onto the image plane.
 * return the number of 3d points in front of the both camera.
 */
static int TriangulateMultiplePointsFromTwoView(const vector<Point2d>& pts1, const vector<Point2d>& pts2, 
					  							 const Mat& Rt1, const Mat& Rt2, const Mat& K,
					  							 vector<Point3d>& result, bool countFront = false)
{
	int count = 0;
	result.clear();
	for (int i=0; i<pts1.size(); i++)
	{
		Point3d point3d;
		bool front = TriangulateSinglePointFromTwoView(pts1[i], pts2[i], Rt1, Rt2, K, point3d, countFront);
		if (front) count++;
		result.push_back(point3d);
	}
	if (!countFront) return 0;
	else return count;
}

static bool ExtractRTfromE(const Mat& E,
						   Mat& R1, Mat& R2, 
						   Mat& t1, Mat& t2)
{
	//Using svd decomposition
	Mat svd_u, svd_vt, svd_w;
	TakeSVD(E,svd_u,svd_vt,svd_w);

	// compute the two possible R and t given the E
	Matx33d W(0,-1,0,
			  1,0,0,
			  0,0,1);
	Matx33d Wt(0,1,0,
			  -1,0,0,
		       0,0,1);
	R1 = svd_u * Mat(W) * svd_vt; //HZ 9.19
	R2 = svd_u * Mat(Wt) * svd_vt; //HZ 9.19
	t1 = svd_u.col(2); //u3
	t2 = -svd_u.col(2); //u3
	return true;
}

/**
 * given the correspondences, compute the fundamental matrix
 *
 * keypoints1 and keypoints2: keypoints extracted from image1 and image2
 * matches: the corresponding relationship between keypoints1 and keypoints2
 * inlierIdx1, inlierIdx2: the aligned inlier idx of keypoints1 and keypoints2.
 */
static void computeFundamentalMatrix(const vector<Point2d>& positions1,
									 const vector<Point2d>& positions2,
									 const vector<DMatch>& matches,
									 vector<Point2d>& inlierPositions1,
									 vector<Point2d>& inlierPositions2,
									 Mat& F,
									 vector<unsigned char>& status)
{
	static const double MAX_DISTANCE = 3.;
	static const double CONFIDENCE = 0.99;

	// construct aligned position arrays
	vector<Point2d> inputs1;
	vector<Point2d> inputs2;
	for (int i=0; i<matches.size(); i++) {
		inputs1.push_back(positions1[matches[i].queryIdx]);
		inputs2.push_back(positions2[matches[i].trainIdx]);
	}

	// fundamental matrix estimation using eight point algorithm with RANSAC
	F = findFundamentalMat(inputs1, inputs2, FM_RANSAC, MAX_DISTANCE, CONFIDENCE, status);

	// construct aligned inlier position arrays
	inlierPositions1.clear();
	inlierPositions2.clear();
	for(int i = 0; i < status.size(); i++) {
		if (status[i]) {
			inlierPositions1.push_back(inputs1[i]);
			inlierPositions2.push_back(inputs2[i]);
		}
	}
}

/**
 * compute essential matrix from fundamental matrix given intrinsics K
 */
static void computeEssentialMatrix(const Mat& F, const Mat& K, Mat& E) {
	E = K.t() * F * K;
}

/**
 * construct 3x4 R|t matrix 
 */
static void constructRt(const Mat& R, const Mat& t, Mat& Rt)
{
	Rt.create(3,4,CV_64F);
	R.copyTo(Rt(Rect(0,0,3,3)));
	Rt.at<double>(0,3) = t.at<double>(0);
	Rt.at<double>(1,3) = t.at<double>(1);
	Rt.at<double>(2,3) = t.at<double>(2);
}

/**
 * A simple feature match routine with ratio test
 * (because ratio test outperforms cross-check)
 */
static void matchFeatures(const Mat& descriptors1, const Mat& descriptors2, vector<DMatch>& matches, float ratio = 0.8) 
{	
	BFMatcher matcher(NORM_HAMMING, false);
	vector<vector<DMatch> > raw_matches;
	matcher.knnMatch(descriptors1, descriptors2, raw_matches, 2);

	// perform ratio test as suggested by by D.Lowe in his paper.
	matches.clear();
	for (int i=0; i < raw_matches.size(); i++) {
		if (raw_matches[i][0].distance < raw_matches[i][1].distance * ratio) {
			matches.push_back(raw_matches[i][0]);
		}
	}
}

/**
 * associate a new feature point with an existing map point
 */
static void associateFeatureWithMapPoint(DataManager& data, int mapPointIndex,
										 Frame& frame, int featureIdx)
{
	frame.features.mapPointsIndices[featureIdx] = mapPointIndex;
	data.mapPoints[mapPointIndex].addObservingFrame(frame.meta.frameID, featureIdx);
}

/**
 * set pose of the reference frame (the first frame).
 */
static void setReferenceFramePose(DataManager& data, int frameIdx)
{
	assert(frameIdx==0);
	data.frames[frameIdx].Rt = Mat::eye(3,4,CV_64F);
}

/**
 * register a new map point and establish the association needed.
 */
static void registerNewMapPoint(DataManager& data, Point3d pos, 
							 	Frame& frame1, int featureIdx1, 
								Frame& frame2, int featureIdx2)
{
	int id = data.mapPoints.size();
	data.mapPoints.push_back(MapPoint(pos, id));
	associateFeatureWithMapPoint(data, id, frame1, featureIdx1);
	associateFeatureWithMapPoint(data, id, frame2, featureIdx2);
}

/**
 * Pose Estimation based on the classical two view reconstruction algorithm.
 *
 * This routine is used for pose estimation for the first two frame
 * by estimating fundamental, essential matrix and extract R/t from
 * essential matrix.
 */
void CameraPoseEstimator::initialPoseEstimation(DataManager& data, int frameIdx)
{
	// fetch references
	assert(frameIdx == 1);
	Frame& curFrame = data.frames[frameIdx];
	Frame& preFrame = data.frames[frameIdx-1];
	vector<Point2d>& positions1 = preFrame.features.positions;
	vector<Point2d>& positions2 = curFrame.features.positions;
	vector<int>& mapPointsIndices1=  preFrame.features.mapPointsIndices;
	vector<int>& mapPointsIndices2=  curFrame.features.mapPointsIndices;
	Mat descriptors1 = preFrame.features.descriptors;
	Mat descriptors2 = curFrame.features.descriptors;
	const Mat& K = data.camera_intrinsics;

	// correspondence matching
	vector<DMatch> matches;
	matchFeatures(descriptors1, descriptors2, matches, FEATURE_MATCH_RATIO_TEST);

#ifdef DEBUG_CameraPoseEstimator
	visualizeFeatureMatching(preFrame.frameBuffer, curFrame.frameBuffer, positions1, positions2, matches);
	waitKey(0);
#endif

	// compute fundamental matrix
	Mat F;
	vector<Point2d> goodPositions1;
	vector<Point2d> goodPositions2;
	vector<unsigned char> status;
	computeFundamentalMatrix(positions1, positions2, matches, goodPositions1, goodPositions2, F, status);
	F.convertTo(F, CV_64F);

#ifdef DEBUG_CameraPoseEstimator
	const Mat& preImge = preFrame.frameBuffer;
	const Mat& curImge = curFrame.frameBuffer;
	drawEpipolarLine(F, preImge, curImge);
#endif

	// compute essential matrix
	Mat E; 
	computeEssentialMatrix(F, K, E);

#ifdef DEBUG_CameraPoseEstimator
	if (!CheckValidEssential(E)) {
		std::cout << "Essential matrix is not valid!" << std::endl;
	}
#endif

	// extract R,t from the essential matrix
	// select two possible (R,t) pairs from the four
	// possible solutions for which the determinant of R is 
	// positive.
	vector<Mat> Rs(4);
	vector<Mat> Ts(4);
	ExtractRTfromE(E,  Rs[0], Rs[1], Ts[0], Ts[1]);
	ExtractRTfromE(Mat(-E), Rs[2], Rs[3], Ts[2], Ts[3]);
	Mat R1, R2, T1, T2;
	for (int i=0; i<4; i++) {
		if(determinant(Rs[i])>0) {
			if (R1.empty()) {
				R1 = Rs[i];
				T1 = Ts[i];
			}
			else if (R2.empty()) {
				R2 = Rs[i];
				T2 = Ts[i];
				break;				
			}
		}
	}

#ifdef DEBUG_CameraPoseEstimator
	if (!CheckValidRotation(R1) || !CheckValidRotation(R2)) {
		std::cout << "Rotation is not valid!" << std::endl;
	}
#endif

	// triangulate both points and only keeps the R|t with which
	// the largest number of points visible in front of the camera.
	Mat Rt1, Rt2;
	constructRt(R1, T1, Rt1);
	constructRt(R2, T2, Rt2);
	Mat I = Mat::eye(3,4, CV_64F);
	vector<Point3d> result1;
	int count0 = TriangulateMultiplePointsFromTwoView(goodPositions1, goodPositions2, I, Rt1, K, result1, true);
	vector<Point3d> result2;
	int count1 = TriangulateMultiplePointsFromTwoView(goodPositions1, goodPositions2, I, Rt2, K, result2, true);

	vector<Point3d>* resultPtr;
	if (count0 > count1) {
		curFrame.Rt = Rt1;
		resultPtr = &result1;
	}
	else {
		curFrame.Rt = Rt2;
		resultPtr = &result2;
	}
	vector<Point3d> &result = *resultPtr;
	// associate each feature points with triangulated points
	int count = 0;
	for (int i=0; i<matches.size(); i++) {
		if (status[i] == 1) {
			registerNewMapPoint(data, result[count], preFrame, matches[i].queryIdx, curFrame, matches[i].trainIdx);
			count ++;
		}
	}
	return;
}

/**
 * Pose Estimation based on perspective-n-points algorithm.
 *
 * This routine is used for pose estimation for the rest all
 * frames, except the first two frames.
 */
void CameraPoseEstimator::pnpPoseEstimation(DataManager& data, int frameIdx)
{
	assert(frameIdx > 1);

	// the number of frames to traverse back to find 3d-2d correspondences and 
	// to triangulate new map points.
	static const int numBackTraverse = 10; 
	
	// traverse in a reverse manner the previous frames
	// and find matched feature points in the previous
	// frames with map points associated.
	vector<Frame>& frames = data.frames;
	Frame &curFrame = frames[frameIdx];
	Features& curFeatures = curFrame.features;
	int numCurFeatures = curFeatures.positions.size();

	vector<bool> matched(numCurFeatures); // record features in the current frame already getting matched to avoid duplicated match.
	vector<Point3d> mapPoints;			  // record 3d point position ...
	vector<Point2f> imagePoints;		  // and corresponding image coordinates.
 
	vector<vector<DMatch> > cachedMatches; // cached the match pairs for triangulation later.
	int count = 0;
	for (int i= frameIdx - 1; i >= 0 && i >= frameIdx - numBackTraverse; i --) 
	{
		Features& preFeatures = frames[i].features;
		vector<DMatch> rawMatches;
		matchFeatures(curFeatures.descriptors, preFeatures.descriptors, rawMatches);
		cachedMatches.push_back(rawMatches);
		for (int j=0; j < rawMatches.size(); j++) 
		{
			int curFrameFeatureIdx = rawMatches[j].queryIdx;
			int preFrameFeatureIdx = rawMatches[j].trainIdx;
			if (preFeatures.mapPointsIndices[preFrameFeatureIdx]!=-1 && !matched[curFrameFeatureIdx]) {
				// associate the feature with the map pointer
				count ++;
				matched[curFrameFeatureIdx] = true;

				int mapPointIdx = preFeatures.mapPointsIndices[preFrameFeatureIdx];
				associateFeatureWithMapPoint(data, mapPointIdx, curFrame, curFrameFeatureIdx);

				// record 3d map point position and 2d image coordinates 
				mapPoints.push_back(data.mapPoints[mapPointIdx].worldPosition);
				imagePoints.push_back(curFeatures.positions[curFrameFeatureIdx]);
				if (count == numCurFeatures) break;
			}
		}
		if (count == numCurFeatures) break;
	}

	// perform perspective-n-point algorithm to 
	// estimate the current camera's pose.
	Mat distCoeff = Mat::zeros(8, 1, CV_64F);
	Mat rvec, tvec, R, Rt;
	solvePnPRansac(mapPoints, imagePoints, data.camera_intrinsics, distCoeff, rvec, tvec);
	Rodrigues(rvec, R); constructRt(R, tvec, Rt);
	curFrame.Rt = Rt;

	// perform triangulation again (but only with the 
	// previous frame) to populate more map points.
	count = 0;
	for (int i= frameIdx - 1; i >= 0 && i >= frameIdx - numBackTraverse; i --)
	{
		Frame& preFrame = data.frames[i];
		Features& preFeatures = frames[i].features;
		int matchIdx = frameIdx -1 - i;
		vector<DMatch>& matches = cachedMatches[matchIdx];
		for (int j=0; j < matches.size(); j++)
		{
			int curFrameFeatureIdx = matches[j].queryIdx;
			int preFrameFeatureIdx = matches[j].trainIdx;	
			// perform triangulation only if the matched features 
			// are both not associated with any map points yet.
			if (preFeatures.mapPointsIndices[preFrameFeatureIdx]==-1 && 
				curFeatures.mapPointsIndices[curFrameFeatureIdx]==-1)
			{
				Point3d pt3d;
				Point2d pts1 = preFeatures.positions[preFrameFeatureIdx];
				Point2d pts2 = curFeatures.positions[curFrameFeatureIdx];
				TriangulateSinglePointFromTwoView(pts1, pts2, preFrame.Rt, curFrame.Rt, data.camera_intrinsics, pt3d);
				registerNewMapPoint(data, pt3d, preFrame, preFrameFeatureIdx, curFrame, curFrameFeatureIdx);
				count ++;
			}		
		}
	}
	return;
}

void CameraPoseEstimator::process(DataManager& data, int frameIdx)
{
	if (frameIdx == 0) {
		setReferenceFramePose(data, frameIdx);
	}
	else if (frameIdx == 1) {
		initialPoseEstimation(data, frameIdx);
	}
	else {
		pnpPoseEstimation(data, frameIdx);
	}
	data.relative_frame_poses.push_back(data.frames[frameIdx].Rt);
	return;
}

bool CameraPoseEstimator::validationCheck(DataManager& data, int frameIdx) {
	return true;
}
