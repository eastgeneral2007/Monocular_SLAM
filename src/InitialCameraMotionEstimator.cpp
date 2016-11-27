#include "InitialCameraMotionEstimator.h"
#include "CommonMath.h"
#ifdef DEBUG_INITIALCAMERAMOTIONESTIMATOR
#include "SFMDebugging.h"
#endif

/**
 * triangulate a single point from two view
 * if the 3d point acquired from triangulation 
 * is in front of the both camera, return true, 
 * else return false.
 */
static bool TriangulateSinglePointFromTwoView(const Point2d& pts1, const Point2d& pts2,  
											  const Mat& Rt1, const Mat& Rt2, const Mat& K,
											  Point3d& result)
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
	Mat pc1 = Rt1*X.t();
	Mat pc2 = Rt2*X.t();

	if (pc1.at<double>(2)>0 && pc2.at<double>(2)>0) return true;
	else return false;
}


/**
 * Perform triangulation, given the camera matrix and keypoints correspondences.
 * We will perform triangulation by minimizing the projection error onto the image plane.
 * return the number of 3d points in front of the both camera.
 */
static int TriangulateMultiplePointsFromTwoView(const vector<Point2d>& pts1, const vector<Point2d>& pts2, 
					  							 const Mat& Rt1, const Mat& Rt2, const Mat& K,
					  							 vector<Point3d>& result)
{
	int count = 0;
	result.clear();
	for (int i=0; i<pts1.size(); i++)
	{
		Point3d point3d;
		bool front = TriangulateSinglePointFromTwoView(pts1[i], pts2[i], Rt1, Rt2, K, point3d);
		if (front) count++;
		result.push_back(point3d);
	}
	return count;
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
									 Mat& F)
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
	vector<unsigned char> status;
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

void InitialCameraMotionEstimator::process(DataManager& data, int frameIdx)
{
	static const int step = 10;
	if (frameIdx - step < 0) {
		return;
	}

	// fetch references
	Frame& curFrame = data.frames[frameIdx];
	Frame& preFrame = data.frames[frameIdx-step];
	vector<Point2d>& positions1 = preFrame.features.positions;
	vector<Point2d>& positions2 = curFrame.features.positions;
	Mat descriptors1 = preFrame.features.descriptors;
	Mat descriptors2 = curFrame.features.descriptors;
	const Mat& K = data.camera_intrinsics;

	// correspondence matching
	vector<DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	// compute fundamental matrix
	Mat F;
	vector<Point2d> goodPositions1;
	vector<Point2d> goodPositions2;
	computeFundamentalMatrix(positions1, positions2, matches, goodPositions1, goodPositions2, F);
	F.convertTo(F, CV_64F);

#ifdef DEBUG_INITIALCAMERAMOTIONESTIMATOR
	const Mat& preImge = preFrame.frameBuffer;
	const Mat& curImge = curFrame.frameBuffer;
	drawEpipolarLine(F, preImge, curImge);
#endif

	// compute essential matrix
	Mat E; 
	computeEssentialMatrix(F, K, E);

#ifdef DEBUG_INITIALCAMERAMOTIONESTIMATOR
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

#ifdef DEBUG_INITIALCAMERAMOTIONESTIMATOR
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
	int count0 = TriangulateMultiplePointsFromTwoView(goodPositions1, goodPositions2, I, Rt1, K, result1);
	vector<Point3d> result2;
	int count1 = TriangulateMultiplePointsFromTwoView(goodPositions1, goodPositions2, I, Rt2, K, result2);

	if (count0 > count1) curFrame.Rt = Rt1;
	else curFrame.Rt = Rt2;

	return;
}

bool InitialCameraMotionEstimator::validationCheck(DataManager& data, int frameIdx)
{
	return true;
}