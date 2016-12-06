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

//  #define DEBUG_CameraPoseEstimator_VisualizeGoodFeatures
//    #define DEBUG_CamercmakaPoseEstimator_SanityCheck
//    #define DEBUG_CameraPoseEstimator_VisualizeMatching
#define DEBUG_CameraPoseEstimator_VisualizeEpipolarline
#define DEBUG_CameraPoseEstimator_ReportReprojectionError
//#define DEBUG_FundamentalMatrix_UsingScratch
#define FILTERING_WITH_F

#include "CameraPoseEstimator.h"
#include "CommonMath.h"
#include "ParamConfig.h"
#include "SFMDebugging.h"
#include "Util.h"
double inlier_threshold = 0.1;
int ransac_iters = 2000;

typedef unsigned char uchar;

// structure for storing matches
struct p_match {
	float   u1p,v1p; // u,v-coordinates in previous left  image
	float   u1c,v1c; // u,v-coordinates in current  left  image
};

extern void printMatrix(const Mat & M, std::string matrix);
// Opencv
static void computeFundamentalMatrix(const vector<Point2d>& positions1,
									 const vector<Point2d>& positions2,
									 const vector<DMatch>& matches,
									 vector<Point2d>& inlierPositions1,
									 vector<Point2d>& inlierPositions2,
									 Mat& F,
									 vector<unsigned char>& status);

// from scratch
static void computeFundamentalMatrix2(const vector<Point2d>& positions1,
									 const vector<Point2d>& positions2,
									 const vector<DMatch>& matches,
									 vector<Point2d>& inlierPositions1,
									 vector<Point2d>& inlierPositions2,
									 Mat& F,
									 vector<uchar>& status);


static double computeReprojectionError(const Point3d& pts3d, const Point2d& pts2d,
									   const Mat& Rt, const Mat& K)
{
	Mat P = K * Rt;
	Mat ptsh = Mat::zeros(4,1,CV_64F);
	ptsh.at<double>(0) = pts3d.x; 
	ptsh.at<double>(1) = pts3d.y;
	ptsh.at<double>(2) = pts3d.z; 
	ptsh.at<double>(3) = 1.;
	Mat pts_rep = P * ptsh;
	pts_rep /= pts_rep.at<double>(2);
	double reprojErr = pow(pts_rep.at<double>(0) - pts2d.x,2) + pow(pts_rep.at<double>(1) - pts2d.y,2);
	return reprojErr;
}

static double computeReprojectionErrorAvg(const vector<Point2d>& pts1, const Mat& Rt1, const Mat& K1, const vector<Point3d>& pts3d)
{
	double totalReprojErr = 0;
	for (int i=0; i<pts3d.size(); i++) {
		totalReprojErr += computeReprojectionError(pts3d[i], pts1[i], Rt1, K1);
	}
	return totalReprojErr/(pts3d.size());
}

/**
 * triangulate a single point from two view
 * if the 3d point acquired from triangulation 
 * is in front of the both camera, return true, 
 * else return false.
 */
static bool TriangulateSinglePointFromTwoView(const Point2d& pts1, const Point2d& pts2,  
											  const Mat& Rt1, const Mat& Rt2, const Mat& K1, const Mat& K2,
											  Point3d& result, bool countFront = false)
{
	// compute camera matrix
	Mat P1 = K1 * Rt1; Mat P2 = K2 * Rt2;

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
		if (pc1.at<double>(2)>0 && pc2.at<double>(2)>0) {
			return true;
		}
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
					  							 const Mat& Rt1, const Mat& Rt2, const Mat& K1, const Mat& K2,
					  							 vector<Point3d>& result, bool countFront = false)
{
	int count = 0;
	result.clear();
	for (int i=0; i<pts1.size(); i++)
	{
		Point3d point3d;
		bool front = TriangulateSinglePointFromTwoView(pts1[i], pts2[i], Rt1, Rt2, K1, K2, point3d, countFront);
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
 * compute essential matrix from fundamental matrix given intrinsics K
 */
static void computeEssentialMatrix(const Mat& F, const Mat& K1, const Mat& K2, Mat& E) {
	E = K2.t() * F * K1;
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
static void setReferenceFramePose(DataManager& data, int frameIdx) {
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
 * concatenate two R|ts
 */
static Mat concatenateRts(Mat Rt1, Mat Rt2) {
	Mat Rt1_4x4 = Mat::zeros(4,4,CV_64F);
	Rt1.copyTo(Rt1_4x4.rowRange(0,3).colRange(0,4));
	Mat Rt2_4x4 = Mat::zeros(4,4,CV_64F);
	Rt2.copyTo(Rt2_4x4.rowRange(0,3).colRange(0,4));
	Mat result = Rt1_4x4 * Rt2_4x4;
	return result.rowRange(0,3).colRange(0,4);
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
	Frame& curFrame = data.frames[frameIdx];
	Frame& preFrame = data.frames[frameIdx-1];
	const Mat& curK = curFrame.K;
	const Mat& preK = preFrame.K;
	vector<Point2d>& positions1 = preFrame.features.positions;
	vector<Point2d>& positions2 = curFrame.features.positions;
	vector<int>& mapPointsIndices1=  preFrame.features.mapPointsIndices;
	vector<int>& mapPointsIndices2=  curFrame.features.mapPointsIndices;
	Mat descriptors1 = preFrame.features.descriptors;
	Mat descriptors2 = curFrame.features.descriptors;

	// correspondence matching
	vector<DMatch> matches;
	matchFeatures(descriptors1, descriptors2, matches, FEATURE_MATCH_RATIO_TEST);

	// compute fundamental matrix
	Mat F;
	vector<Point2d> goodPositions1;
	vector<Point2d> goodPositions2;
	
	vector<unsigned char> status;
	#ifdef DEBUG_FundamentalMatrix_UsingScratch
	computeFundamentalMatrix2(positions1, positions2, matches, goodPositions1, goodPositions2, F, status);
	#else
	computeFundamentalMatrix(positions1, positions2, matches, goodPositions1, goodPositions2, F, status);
	F.convertTo(F, CV_64F);
	#endif
#ifdef DEBUG_CameraPoseEstimator_VisualizeMatching
	vector<DMatch> selectedMatches;
	for (int i=0; i<(int)status.size(); i++) {
		if (status[i]) {
			selectedMatches.push_back(matches[i]);
		}
	}
	visualizeFeatureMatching(preFrame.frameBuffer, curFrame.frameBuffer, positions1, positions2, selectedMatches);
#endif

#ifdef DEBUG_CameraPoseEstimator_VisualizeGoodFeatures
	visualizeFeature(curFrame.frameBuffer, goodPositions2);
#endif

#ifdef DEBUG_CameraPoseEstimator_VisualizeEpipolarline
	const Mat& preImge = preFrame.frameBuffer;
	const Mat& curImge = curFrame.frameBuffer;
	drawEpipolarLine(F, preImge, curImge);
#endif

	// compute essential matrix
	Mat E; 
	computeEssentialMatrix(F, preK, curK, E);

#ifdef DEBUG_CameraPoseEstimator_SanityCheck
	if (!CheckValidEssential(E)) {
		std::cout << "Essential matrix is not valid!" << std::endl;
	}
#endif

	// extract R/t from E.

	Mat R1, R2, T1, T2;
	ExtractRTfromE(E, R1, R2, T1, T2);
	if (abs(determinant(R1) + 1.0) < EPSILON) {
		E = -E;
		ExtractRTfromE(E, R1, R2, T1, T2);
	}
#ifdef DEBUG_CameraPoseEstimator_SanityCheck
	if (!CheckValidRotation(R1) || !CheckValidRotation(R2)) {
		std::cout << "Rotation is not valid!" << std::endl;
	}
#endif
	Mat Rts[4];
	constructRt(R1, T1, Rts[0]); constructRt(R1, T2, Rts[1]);
	constructRt(R2, T1, Rts[2]); constructRt(R2, T2, Rts[3]);
	Mat I = Mat::eye(3,4, CV_64F);
	int bestRtIndex = -1;
	int maxCount = -1;
	vector<Point3d> result;
	for (int i=0; i < 4; i++) {
		vector<Point3d> tmpResult;
		Mat Rt = Rts[i];
		int count = TriangulateMultiplePointsFromTwoView(goodPositions1, goodPositions2, I, Rt, preK, curK, tmpResult, true);
		cout << count << " of " << goodPositions1.size() << " 3D points are in front of the camera." << endl;
		if (maxCount < count) {
			maxCount = count;
			bestRtIndex = i;
			result = tmpResult;
		}
	}
	curFrame.Rt = Rts[bestRtIndex];
	curFrame.Rt = concatenateRts(preFrame.Rt, curFrame.Rt);

#ifdef DEBUG_CameraPoseEstimator_ReportReprojectionError
	double reprojErr = 0;
	reprojErr += computeReprojectionErrorAvg(goodPositions1, I, preK, result);
	reprojErr += computeReprojectionErrorAvg(goodPositions2, curFrame.Rt, curK, result);
	reprojErr /= 2; 
	cout << "reprojection err: " << reprojErr << endl;
#endif

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
	// the number of frames to traverse back to find 3d-2d correspondences and 
	// to triangulate new map points.
	static const int numBackTraverse = 5; 
	
	// traverse in a reverse manner the previous frames
	// and find matched feature points in the previous
	// frames with map points associated.
	vector<Frame>& frames = data.frames;
	Frame &curFrame = frames[frameIdx];
	Features& curFeatures = curFrame.features;
	const Mat& curK = curFrame.K;
	int numCurFeatures = curFeatures.positions.size();

	vector<bool> matched(numCurFeatures); // record features in the current frame already getting matched to avoid duplicated match.
	vector<Point3d> mapPoints;			  // record 3d point position ...
	vector<Point2d> imagePoints;		  // and corresponding image coordinates.
 
	vector<vector<DMatch> > cachedMatches; // cached the match pairs for triangulation later.
	int count = 0;
	for (int i= frameIdx - 1; i >= 0 && i >= frameIdx - numBackTraverse; i --) 
	{
		Features& preFeatures = frames[i].features;
		vector<DMatch> rawMatches;
		matchFeatures(curFeatures.descriptors, preFeatures.descriptors, rawMatches);

		// use epipolar constraint to filter out good matches
#ifdef FILTERING_WITH_F
		cout << rawMatches.size() << endl;
		vector<Point2d> tmp0, tmp1; Mat tmp2;
		vector<unsigned char> status;
		#ifdef DEBUG_FundamentalMatrix_UsingScratch
		computeFundamentalMatrix2(curFeatures.positions, preFeatures.positions, rawMatches, tmp0, tmp1, tmp2, status);
		#else
		computeFundamentalMatrix(curFeatures.positions, preFeatures.positions, rawMatches, tmp0, tmp1, tmp2, status);
		#endif
		int count0 = 0;
		for(int j=0; j<status.size(); j++) {
			if (status[j]) rawMatches[count0++] = rawMatches[j];
		}
		rawMatches.resize(count0);
		// cout << "removed " << (int)status.size() - count0 << " outliers" << endl;
#endif
		cachedMatches.push_back(rawMatches);
#ifdef DEBUG_CameraPoseEstimator_VisualizeMatching
		vector<DMatch> selectedMatches;
#endif
		for (int j=0; j < rawMatches.size(); j++) 
		{
			int curFrameFeatureIdx = rawMatches[j].queryIdx;
			int preFrameFeatureIdx = rawMatches[j].trainIdx;
			if (preFeatures.mapPointsIndices[preFrameFeatureIdx]!=-1 && !matched[curFrameFeatureIdx]) {
				// associate the feature with the map pointer
				count ++;
				matched[curFrameFeatureIdx] = true;

#ifdef DEBUG_CameraPoseEstimator_VisualizeMatching
			    selectedMatches.push_back(rawMatches[j]);
#endif
				int mapPointIdx = preFeatures.mapPointsIndices[preFrameFeatureIdx];
				associateFeatureWithMapPoint(data, mapPointIdx, curFrame, curFrameFeatureIdx);

				// record 3d map point position and 2d image coordinates 
				mapPoints.push_back(data.mapPoints[mapPointIdx].worldPosition);
				imagePoints.push_back(curFeatures.positions[curFrameFeatureIdx]);
				if (count == numCurFeatures) break;
			}
		}

#ifdef DEBUG_CameraPoseEstimator_VisualizeMatching
		visualizeFeatureMatching(curFrame.frameBuffer, data.frames[i].frameBuffer, curFeatures.positions, preFeatures.positions, rawMatches);
#endif
		if (count == numCurFeatures) break;
	}

	// perform perspective-n-point algorithm to 
	// estimate the current camera's pose.
	Mat distCoeff = Mat::zeros(8, 1, CV_64F);

	// for F1
	distCoeff.at<double>(0) = 0.2624;
	distCoeff.at<double>(1) = -0.9531;
	distCoeff.at<double>(2) = -0.0054;
	distCoeff.at<double>(3) = 0.0026;
	distCoeff.at<double>(4) = 1.1633;

	Mat rvec, tvec, R, Rt;
	solvePnPRansac(mapPoints, imagePoints, curK, distCoeff, rvec, tvec);
	Rodrigues(rvec, R); constructRt(R, tvec, Rt);
	curFrame.Rt = Rt;

#ifdef DEBUG_CameraPoseEstimator_ReportReprojectionError
	double reprojErr = 0;
	reprojErr += computeReprojectionErrorAvg(imagePoints, curFrame.Rt, curK, mapPoints);
	cout << "reprojection err: " << reprojErr << endl;
#endif

	// invoke pose Bundle Adjustment here
	// Util::PoseBundleAdjustment(curFrame, data);

	// perform triangulation again (but only with the 
	// previous frame) to populate more map points.
  	count = 0;
	for (int i= frameIdx - 1; i >= 0 && i >= frameIdx - numBackTraverse; i --)
	{
		Frame& preFrame = data.frames[i];
		const Mat& preK = preFrame.K;
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
				TriangulateSinglePointFromTwoView(pts1, pts2, preFrame.Rt, curFrame.Rt, preK, curK, pt3d);
				registerNewMapPoint(data, pt3d, preFrame, preFrameFeatureIdx, curFrame, curFrameFeatureIdx);
				count ++;
			}		
		}
	}

	return;
}

void CameraPoseEstimator::process(DataManager& data, int frameIdx)
{
	if (frameIdx == 0 ) {
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
	F = findFundamentalMat(inputs1, inputs2, CV_FM_RANSAC, MAX_DISTANCE, CONFIDENCE, status);

	// construct aligned inlier position arrays
	inlierPositions1.clear();
	inlierPositions2.clear();
	for(int i = 0; i < status.size(); i++) {
		if (status[i]) {
			inlierPositions1.push_back(inputs1[i]);
			inlierPositions2.push_back(inputs2[i]);
		}
	}

	// use the inliers and compute F again
	vector<Point2d> newInputs1;
	vector<Point2d> newInputs2;
	vector<DMatch> selectedMatches;
	for (int i=0; i<status.size(); i++) {
		if (status[i]) {
			newInputs1.push_back(inputs1[i]);
			newInputs2.push_back(inputs2[i]);
		}
	}
	F = findFundamentalMat(newInputs1, newInputs2, CV_FM_8POINT);
}



////////////////////////////////// estimate F //////////////////////////////////

void fundamentalMatrix(const vector<p_match> &p_matched, const vector<uchar> &active, Mat &F);
vector<uchar> getRandomSample(int N,int num);
static vector<uchar> getInlier (vector<p_match> &p_matched, Mat &F);

static void computeFundamentalMatrix2(const vector<Point2d>& positions1,
									 const vector<Point2d>& positions2,
									 const vector<DMatch>& matches,
									 vector<Point2d>& inlierPositions1,
									 vector<Point2d>& inlierPositions2,
									 Mat& F,
									 vector<unsigned char>& status)
{  	
	// number of active p_matched
	int N = (int)matches.size();
	// construct aligned position arrays
	vector<Point2f> inputs1;
	vector<Point2f> inputs2;
	double maxt = 0;
	for (int i=0; i<(int)matches.size(); i++) {
		maxt = MAX(maxt, MAX(positions1[matches[i].queryIdx].x, positions2[matches[i].trainIdx].x));
		maxt = MAX(maxt, MAX(positions1[matches[i].queryIdx].y, positions2[matches[i].trainIdx].y));
	}

	for (int i=0; i<(int)matches.size(); i++) {
		Point2f p1, p2;
		p1.x = (double)positions1[matches[i].queryIdx].x / maxt;
		p1.y = (double)positions1[matches[i].queryIdx].y / maxt;
		p2.x = (double)positions1[matches[i].trainIdx].x / maxt;
		p2.y = (double)positions1[matches[i].trainIdx].y / maxt;
		inputs1.push_back(p1);
		inputs2.push_back(p2);
	}

	// create constraint matrix A
	vector<p_match> p_matched;
	for (int i=0; i<N; i++) {
		p_match m;
		m.u1p = inputs1[i].x;
		m.v1p = inputs1[i].y;
		m.u1c = inputs2[i].x;
		m.v1c = inputs2[i].y;
		p_matched.push_back(m);
	}

	// initial RANSAC estimate of F
	vector<unsigned char> inliers;
	inliers.clear();
	for (int k=0;k<ransac_iters;k++) {
		// draw random sample set	
		vector<uchar> active = getRandomSample(N,8);
		fundamentalMatrix(p_matched, active, F);
		vector<uchar> inliers_curr = getInlier(p_matched,F);

		// update model if we are better
		if (inliers_curr.size()>inliers.size())
			inliers = inliers_curr;
	}
	
	// are there enough inliers?
	assert(inliers.size()>10 && "Not enough inliers when estimating F & RANSAC. ");
	
	// construct aligned inlier position arrays
	inlierPositions1.clear();
	inlierPositions2.clear();
	status.clear();
	status.resize(N, 0);
	for(int i = 0; i < inliers.size(); i++) {
		status[inliers[i]]=1;
		inlierPositions1.push_back(inputs1[inliers[i]]);
		inlierPositions2.push_back(inputs2[inliers[i]]);
	}
	
	// refine F using all inliers
	fundamentalMatrix(p_matched, inliers, F); 
	double times[3][3] = {{1.0/maxt,0,0},{0,1.0/maxt,0},{0,0,1.0}};
	Mat times_F = Mat(3,3,CV_64F, times);
	Mat res = times_F.t() * F * times_F;
	res.copyTo(F);
}

void fundamentalMatrix(const vector<p_match> &p_matched, const vector<uchar> &active, Mat &F)
{
	int N = active.size();
	Mat A(N,9, CV_64F);
	for (int i=0; i<N; i++) {
		p_match m = p_matched[active[i]];
		A.at<double>(i,0) = m.u1c*m.u1p;
		A.at<double>(i,1) = m.u1c*m.v1p;
		A.at<double>(i,2) = m.u1c;
		A.at<double>(i,3) = m.v1c*m.u1p;
		A.at<double>(i,4) = m.v1c*m.v1p;
		A.at<double>(i,5) = m.v1c;
		A.at<double>(i,6) = m.u1p;
		A.at<double>(i,7) = m.v1p;
		A.at<double>(i,8) = 1;
	}
   
  // compute singular value decomposition of A
	Mat U,W,Vt;
	TakeSVD(A,U,Vt,W);
	//   A.svd(U,W,V);

	Mat V = Vt.t();
	// extract fundamental matrix from the column of V corresponding to the smallest singular value
	F = Mat::zeros(3,3,CV_64F);
	F.at<double>(0,0) = V.at<double>(0,8);
	F.at<double>(0,1) = V.at<double>(1,8);
	F.at<double>(0,2) = V.at<double>(2,8);
	F.at<double>(1,0) = V.at<double>(3,8);
	F.at<double>(1,1) = V.at<double>(4,8);
	F.at<double>(1,2) = V.at<double>(5,8);
	F.at<double>(2,0) = V.at<double>(6,8);
	F.at<double>(2,1) = V.at<double>(7,8);
	F.at<double>(2,2) = V.at<double>(8,8);
	// printMatrix(F,"F");

	// enforce rank 2
	TakeSVD(F,U,Vt,W);
	//   F.svd(U,W,V);
	W.at<double>(2,0) = 0;
	F = U*Mat::diag(W)*Vt;
}

vector<uchar> getInlier (vector<p_match> &p_matched, Mat &F) {
  // extract fundamental matrix
  double f00 = F.at<double>(0,0); double f01 = F.at<double>(0,1); double f02 = F.at<double>(0,2);
  double f10 = F.at<double>(1,0); double f11 = F.at<double>(1,1); double f12 = F.at<double>(1,2);
  double f20 = F.at<double>(2,0); double f21 = F.at<double>(2,1); double f22 = F.at<double>(2,2);
  
  // loop variables
  double u1,v1,u2,v2;
  double x2tFx1;
  double Fx1u,Fx1v,Fx1w;
  double Ftx2u,Ftx2v;
  
  // vector with inliers
  vector<uchar> inliers;
  
  // for all matches do
  for (int i=0; i<p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;
    
    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;
    
    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;
    
    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;
    
	// sampson distance
	double d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);
	#ifdef DEBUG_CameraPoseEstimator_VisualizeEpipolarline
	//cout << fabs(d)<< "   ";
	#endif
	// check threshold
	if (fabs(d) < inlier_threshold)
		inliers.push_back(i);
}

// return set of all inliers
return inliers;
}


vector<uchar> getRandomSample(int N,int num) {

  // init sample and totalset
  vector<uchar> sample;
  vector<uchar> totalset;
  
  // create vector containing all indices
  for (int i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  for (int i=0; i<num; i++) {
	int j = rand()%totalset.size();
	sample.push_back(totalset[j]);
	totalset.erase(totalset.begin()+j);
  }
  
  // return sample
  return sample;
}