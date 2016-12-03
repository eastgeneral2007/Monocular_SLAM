//
// PCLUtils.cpp
//
// A bunch of utility functions related
// to the PCL library.
// 
// @Yu

#include "PCLUtils.h"

/*
 * Convert From vector<cv::Point3d> to pcl::PointCloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(vector<cv::Point3d>& opencvPointCloud)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

     for(int i=0;i<opencvPointCloud.size();i++)
     {
        pcl::PointXYZ point;
        point.x = opencvPointCloud[i].x;
        point.y = opencvPointCloud[i].y;
        point.z = opencvPointCloud[i].z;
        point_cloud_ptr->points.push_back(point);

     }
     point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
     point_cloud_ptr->height = 1;
     return point_cloud_ptr;
}
