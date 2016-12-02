//
// SFMDebugging.cpp
//
// a bunch of GUI debugging tools for Structure 
// from Motion.
//
// @Yu

#include "Common.h"
#include "SFMDebugging.h"
#include "GUIhelper.h"

struct DrawEpipolarLineUserData
{
	Mat* img1;
	Mat* img2;
	const Mat* F;
	DrawEpipolarLineUserData(Mat* img1, Mat* img2, const Mat *F): img1(img1), img2(img2), F(F) {}
};

static void drawEpipolarLineCallback(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
    {
		DrawEpipolarLineUserData* userData = (DrawEpipolarLineUserData*)userdata;
		const Mat& F = *(userData->F);
		Mat& img1 = *(userData->img1);
		Mat& img2 = *(userData->img2);
		int width1 = img1.cols - 1;
		int height1 = img1.rows - 1;
		if (x < width1 && y < height1)
		{
			Matx31d x0(x,y,1);
			Mat abc = F * Mat(x0);
			drawLine(img2, abc.at<double>(0), abc.at<double>(1), abc.at<double>(2));
			drawPoint(img1, Point(x,y));
			imshow2("visualization",img1, img2);
			waitKey();
		}
    }
}

/**
 * visualizeFeatureMatching
 *
 * visualize the feature matching correspondence
 * of two frames.
 */
 void visualizeFeatureMatching(const Mat& img1, const Mat& img2, 
 							   const vector<Point2d>& pos1, const vector<Point2d>& pos2, 
							   const vector<DMatch>& matches)
 {
	 vector<KeyPoint> keypoints1;
	 for (int i = 0; i < pos1.size(); i++) {
		 keypoints1.push_back(KeyPoint(pos1[i].x, pos1[i].y, 1));
	 }
	 vector<KeyPoint> keypoints2;
	 for (int i = 0; i < pos1.size(); i++) {
		 keypoints2.push_back(KeyPoint(pos2[i].x, pos2[i].y, 1));
	 }
	 Mat outImg;
	 drawMatches(img1, keypoints1, img2, keypoints2, matches, outImg);
	 imshow("feature matching", outImg);
 }


/**
 * drawEpipolarLine
 * 
 * The usr click a point on the right image, the corresponding 
 * epipolar line in the right image will be drawn.
 */
void drawEpipolarLine(const Mat& F, const Mat& img1, const Mat& img2)
{
	Mat img1_visualization = img1.clone();
	Mat img2_visualization = img2.clone();
	imshow2("visualization",img1_visualization, img2_visualization);
	DrawEpipolarLineUserData userData(&img1_visualization, &img2_visualization, &F);
	setMouseCallback("visualization", drawEpipolarLineCallback, &userData);
	waitKey();
}

/**
 * check whether a rotation matrix is valid (determinant =1)
 */
bool CheckValidRotation(const Mat& rotation) {
	return fabs(determinant(rotation) - 1) < EPSILON;
}

/**
 * check whether an essential matrix is valid (whether it is singular matrix)
 */
bool CheckValidEssential(const Mat& essential) {
	return fabs(determinant(essential)) < EPSILON;
}

