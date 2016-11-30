//
// PCLUtils.h
//
// A bunch of utility functions related
// to the PCL library.
// 
// @Yu

#include "Common.h"
#include "CommonPCL.h"
#include "CommonCV.h"

/*
 * Convert From vector<Point3d> to pcl::PointCloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(vector<Point3d>& opencvPointCloud);