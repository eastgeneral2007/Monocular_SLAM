//
// GUIhelper.cpp
//
// A bunch of wrapper functions for 
// OpenCV GUI operations.
//
// @Yu

#include "CommonCV.h"

void imshow2(const char* title, const Mat& im1, const Mat& im2)
{
  Size sz1 = im1.size();
  Size sz2 = im2.size();
  Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
  im1.copyTo(im3(Rect(0, 0, sz1.width, sz1.height)));
  im2.copyTo(im3(Rect(sz1.width, 0, sz2.width, sz2.height)));
  imshow(title, im3);
  return;
}

void drawLine(Mat img, Point start, Point end, Scalar color)
{
  const static int thickness = 2;
  const static int lineType = 8;
  line(img,
       start,
       end,
       color,
       thickness,
       lineType );
  return;
}

void drawPoint(Mat img, Point position, Scalar color)
{
  const int thickness = -1;
  const int lineType = 8;
  int w = img.cols;
  circle(img,
         position,
         w/128.0,
         color,
         thickness,
         lineType);
  return;
}

void drawLine(Mat img, double a, double b, double c, Scalar color)
{
  int y0 = 0;
  int y1 = img.rows-1;
  int x0 = -(b*y0+c)/a;
  int x1 = -(b*y1+c)/a;
  drawLine(img, Point(x0,y0), Point(x1, y1), color);
  return;
}


