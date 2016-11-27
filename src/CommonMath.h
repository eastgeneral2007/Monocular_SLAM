#ifndef COMMONMATH_H
#define COMMONMATH_H

#include "CommonCV.h"

/**
 * a convinent wrapper for computing svd
 */
void TakeSVD(const Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w);

/**
 * solve for x the homogeneous least square problem: Ax = 0
 */
void solveHLS(const Mat& A, Mat& x);

/**
 * dehomogenize a 4-component array
 */
void dehomogenize(Mat& X);

/**
 * convert from 3-element mat to point3d
 */
void Mat2Point3d(const Mat& mat, Point3d& point3d);


#endif