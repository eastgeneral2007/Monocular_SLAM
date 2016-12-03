//
// PointCloudVisualizer.cpp
//
// PointCloudVisualizer is a visualization tool 
// for rendering the result of 3D reconstruction.
//
// @Yu, Sharon

// #define DEBUG_POINTCLOUD_VISUALIZER

#include "PointCloudVisualizer.h"
#include "PCLUtils.h"
#include <pcl/common/common_headers.h>
using namespace pcl;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr    CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudRGB;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> VisPtr;

const static char* TITLE_NAME = "3D Visualizer";
const static char* CLOUD_NAME = "map points";

static CloudPtr generateTestCloud();
static void renderPointCloud(VisPtr viewer, CloudRGB cloud);
static VisPtr createVisualizer();
static CloudPtr MapPointsToCloudPtr(const vector<MapPoint>& points);
static CloudRGB MapPointsToCloudRGB(DataManager& data, int frameIdx);
static void CamPosToCloudRGB(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static PolygonMesh PossionReconstruction(CloudPtr cloud);

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
#else 
    CloudRGB cloudMapPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    // TODO:: plot point cloud 
    // cloudMapPoints = MapPointsToCloudRGB(data, frameIdx);
    if (cloudMapPoints->width>20)
    {
        // pcl::PolygonMesh triangles = PossionReconstruction(cloud);
        // viewer->addPolygonMesh(triangles);
    }
    // TODO:: plot camera trajectory
    CamPosToCloudRGB(viewer, data, frameIdx, cloudMapPoints);
    renderPointCloud(viewer, cloudMapPoints);
#endif
    viewer->spinOnce (10);
}

bool PointCloudVisualizer::validationCheck(DataManager& data, int frameIdx)
{
    return true;
}

static VisPtr createVisualizer()
{
    VisPtr viewer = VisPtr(new pcl::visualization::PCLVisualizer (TITLE_NAME));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    viewer->addCoordinateSystem (1);
    viewer->initCameraParameters ();
    return (viewer);
}

// render point cloud
static void renderPointCloud(VisPtr viewer, CloudRGB cloud)
{
    if (viewer->contains(CLOUD_NAME)) {
        viewer->updatePointCloud(cloud, CLOUD_NAME);
    }
    else {
        viewer->addPointCloud(cloud, CLOUD_NAME);
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, CLOUD_NAME);
}

#ifdef DEBUG_POINTCLOUD_VISUALIZER
static CloudPtr generateTestCloud()
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

static void CamPosToCloudRGB(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr)
{
    pcl::PointXYZRGB basic_point, pre_point;
    for (int i=0; i<frameIdx; i++)
    {
        pre_point = basic_point;
        basic_point.x = data.frames[i].Rt.at<float>(0,3);
        basic_point.y = data.frames[i].Rt.at<float>(1,3);
        basic_point.z = data.frames[i].Rt.at<float>(2,3);
        basic_point.r = 220;
        basic_point.g = 30;
        basic_point.b = 30;
        cout<<"Camera pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
        basic_cloud_ptr->points.push_back(basic_point);
    }
    if (frameIdx > 1)
        viewer->addLine(pre_point, basic_point, to_string(frameIdx), 0);
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
}


static CloudPtr MapPointsToCloudPtr(const vector<MapPoint>& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<points.size(); i++)
    {
        const MapPoint &point = points[i];
        pcl::PointXYZ basic_point;
        basic_point.x = point.worldPosition.x;
        basic_point.y = point.worldPosition.y;
        basic_point.z = point.worldPosition.z;
        // cout<<"pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
        basic_cloud_ptr->points.push_back(basic_point);
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    return basic_cloud_ptr;    
}

static CloudRGB MapPointsToCloudRGB(DataManager& data, int frameIdx)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i<data.mapPoints.size(); i++)
    {
        const MapPoint &point = data.mapPoints[i];
        pcl::PointXYZRGB basic_point;
        basic_point.x = point.worldPosition.x;
        basic_point.y = point.worldPosition.y;
        basic_point.z = point.worldPosition.z;
        basic_point.r = 30;
        basic_point.g = 30;
        basic_point.b = 220;
        // cout<<"pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
        basic_cloud_ptr->points.push_back(basic_point);
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    return basic_cloud_ptr;    
}


static PolygonMesh PossionReconstruction(CloudPtr cloud)
{
    cout << "num of pts: " << cloud->width << endl;
    cout << "begin passthrough filter" << endl;
    pcl::PointCloud<PointXYZ>::Ptr filtered(new pcl::PointCloud<PointXYZ>());
    pcl::PassThrough<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);
    cout << "passthrough filter complete" << endl;
    cout << "num of pts: " << filtered->width << endl;

    
    cout << "begin moving least squares" << endl;
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(10);//0.01
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(10); //0.005
    mls.setUpsamplingStepSize(5);//0.03

    PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
    mls.process(*cloud_smoothed);
    cout << "MLS complete" << endl;
    cout << "num of pts: " << cloud_smoothed->width << endl;

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (5); //0.025

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    return triangles;
    // Finish
}