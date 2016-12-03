//
// PCLUtils.h
//
// A bunch of utility functions related
// to the PCL library.
// 
// @Yu

#include "Common.h"
#include "CommonPCL.h"
// #include "CommonCV.h"
#include "opencv2/opencv.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/core/core.hpp"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


/*
 * Convert From vector<Point3d> to pcl::PointCloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(vector<cv::Point3d>& opencvPointCloud);