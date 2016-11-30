//
// GUIhelper.h
//
// A bunch of wrapper functions for 
// OpenCV GUI operations.
//
// @Yu


#include "CommonCV.h"

void imshow2(const char* title, const Mat& im1, const Mat& im2);
void drawPoint(Mat img, Point position, Scalar color = Scalar(255,0,0));
void drawLine(Mat img, Point start, Point end, Scalar color = Scalar(0,0,255));
void drawLine(Mat img, double a, double b, double c, Scalar color = Scalar(0,0,255));