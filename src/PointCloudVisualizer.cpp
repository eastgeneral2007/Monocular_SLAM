//
// PointCloudVisualizer.cpp
//
// PointCloudVisualizer is a visualization tool 
// for rendering the result of 3D reconstruction.
//
// @Yu

#define DEBUG_POINTCLOUD_VISUALIZER

#include "PointCloudVisualizer.h"
#include "PCLUtils.h"
#include <pcl/common/common_headers.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> VisPtr;

const static char* TITLE_NAME = "3D Visualizer";
const static char* CLOUD_NAME = "map points";

static CloudPtr generateTestCloud();
static void renderPointCloud(VisPtr viewer, CloudPtr cloud);
static VisPtr createVisualizer();

#ifdef DEBUG_POINTCLOUD_VISUALIZER
static CloudPtr cloud;
#endif

void PointCloudVisualizer::init()
{
#ifdef DEBUG_POINTCLOUD_VISUALIZER
    cloud = generateTestCloud();
#endif
    viewer = createVisualizer();
}

void PointCloudVisualizer::process(DataManager& data, int frameIdx)
{
#ifdef DEBUG_POINTCLOUD_VISUALIZER
    renderPointCloud(viewer, cloud);
#endif
    viewer->spinOnce (100);
}

bool PointCloudVisualizer::validationCheck(DataManager& data, int frameIdx)
{
    return true;
}

static boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer (TITLE_NAME));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    // viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

static void renderPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (viewer->contains(CLOUD_NAME)) {
        viewer->updatePointCloud(cloud, CLOUD_NAME);
    }
    else {
        viewer->addPointCloud(cloud, CLOUD_NAME);
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, CLOUD_NAME);
}

#ifdef DEBUG_POINTCLOUD_VISUALIZER
static pcl::PointCloud<pcl::PointXYZ>::Ptr generateTestCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);
        }
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    return basic_cloud_ptr;
}
#endif