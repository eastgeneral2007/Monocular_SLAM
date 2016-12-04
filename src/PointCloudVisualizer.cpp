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
static void CamPosToCloudRGBWithGT(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static PolygonMesh PossionReconstruction(CloudPtr cloud);
void printMatrix(const cv::Mat &M, std::string matrix);
void RtToWorldT(const Mat &Rt, Mat &t_res);

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
    CloudRGB cloudMapPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    // To plot the scene only
    cloudMapPoints = MapPointsToCloudRGB(data, frameIdx);
    
    // TODO:: Mesh reconstruction from point clouds
    if (cloudMapPoints->width>20)
    {
        // pcl::PolygonMesh triangles = PossionReconstruction(cloud);
        // viewer->addPolygonMesh(triangles, "polygon", 0);
    }

    // To plot the camera trajectory only
    // CamPosToCloudRGB(viewer, data, frameIdx, cloudMapPoints);        // without ground truth R|t
    CamPosToCloudRGBWithGT(viewer, data, frameIdx, cloudMapPoints);     // with ground truth R|t
    renderPointCloud(viewer, cloudMapPoints);
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

void printMatrix(const cv::Mat &M, std::string matrix)
{
    printf("Matrix \"%s\" is %i x %i\n", matrix.c_str(), M.rows, M.cols);
    std::cout << M << std::endl;
}

void RtToWorldT(const Mat &Rt, Mat &t_res)
{
    Mat R,t;
    Rt(Range(0,3),Range(0,3)).copyTo(R);
    Rt(Range(0,3),Range(3,4)).copyTo(t);
    Mat t_new = R.inv()*t;
    t_new.copyTo(t_res);
    //printMatrix(R, "R");
    //printMatrix(t, "t");
    //printMatrix(t_res, "t_world");
}

static void CamPosToCloudRGB(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr)
{
    pcl::PointXYZRGB basic_point, pre_point;
    for (int i=0; i<=frameIdx; i++)
    {
        pre_point = basic_point;
        Mat t;
        RtToWorldT(data.frames[i].Rt, t);
        basic_point.x = (float)t.at<double>(0,0);
        basic_point.y = (float)t.at<double>(1,0);
        basic_point.z = (float)t.at<double>(2,0);
        basic_point.r = 220;
        basic_point.g = 30;
        basic_point.b = 30;
        basic_cloud_ptr->points.push_back(basic_point);
    }
    cout<<"Camera pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
    if (frameIdx > 0)
    {
        viewer->addLine(pre_point, basic_point, 80, 20, 20, to_string(frameIdx), 0);
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
}

static void CamPosToCloudRGBWithGT(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr)
{
    pcl::PointXYZRGB basic_point, pre_point, basic_point_gt, pre_point_gt;
    for (int i=0; i<=frameIdx; i++)
    {
        pre_point = basic_point;
        Mat Rt = data.frames[i].Rt;
        //double M1[3][4] = {{1,0,0,0},{0,0.5, 0.8660,0},{0,-0.8660, 0.5, 1}};
        //double M1[3][4] = {{0.5000, 0.8660, 0,0},{ -0.8660  ,  0.5000 ,0,0},{0,0,1, 1}};
        //double M2[3][4] = {{0.500,0,   -0.8660,0},{ 0  ,  1.0000,         0,-1},{0.8660 ,        0,    0.5000,0}};
        //Mat Rt = Mat(3, 4, CV_64F, M1);
        Mat t;
        RtToWorldT(Rt,t);
        
        basic_point.x = t.at<double>(0,0);
        basic_point.y = t.at<double>(1,0);
        basic_point.z = t.at<double>(2,0);
        basic_point.r = 220;
        basic_point.g = 30;
        basic_point.b = 30;
        basic_cloud_ptr->points.push_back(basic_point);

        pre_point_gt = basic_point_gt;
        basic_point_gt.x = data.frames[i].RtGt.at<double>(0,3);
        basic_point_gt.y = data.frames[i].RtGt.at<double>(1,3);
        basic_point_gt.z = data.frames[i].RtGt.at<double>(2,3);
        basic_point_gt.r = 30;
        basic_point_gt.g = 220;
        basic_point_gt.b = 30;
        basic_cloud_ptr->points.push_back(basic_point_gt);
    }
    cout<<"Camera pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
    if (frameIdx > 0)
    {
        viewer->addLine(pre_point, basic_point, 250, 20, 20, to_string(frameIdx), 0);
        viewer->addLine(pre_point_gt, basic_point_gt, 20, 250, 20, "gt"+to_string(frameIdx), 0);
    }
 
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