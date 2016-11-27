#include "CommonMath.h"

void TakeSVD(const Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w) {
	//Using OpenCV's SVD
	SVD svd(E,SVD::MODIFY_A);
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;
	return;
}

void solveHLS(const Mat& A, Mat& x) {
	SVD svd(A, SVD::MODIFY_A);
	Mat vt = svd.vt;
	x = vt.row(vt.rows-1);
	return;
}

void dehomogenize(Mat& X)
{
	X/=X.at<double>(3);
}

void Mat2Point3d(const Mat& mat, Point3d& point3d)
{
	point3d.x = mat.at<double>(0);
	point3d.y = mat.at<double>(1);
	point3d.z = mat.at<double>(2);
}
