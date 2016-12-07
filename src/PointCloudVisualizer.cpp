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
#include "Util.h"
using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBNormal PointTypeN;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr    CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudRGB;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> VisPtr;


///////////////////////////////////////////// Visualization Options /////////////////////////////////////////////
#define ShowOrbSlam               // Show ORB SLAM point cloud 
// #define ShowGroundTruth             // Show ground truth point cloud 
#define ShowCameraTrajectory        // When visualizing point cloud, show trajectory & cameras
//#define OnlyTrajectory                // don't show point cloud

#define drawCameraPyramid
#define PlotAllFrames             // To accumulate or show single frame

double depth_density_ratio = 0.05;   // depth map downsampling ratio [0, 1]:   0 no points,  1 original

// #define ShowMeshReconstruction      // To perform mesh reconstruction
int displayForm = 0;             // Mesh representation: 0 for Wireframe(standard),  1 for surface, 2 for Points
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const static char* TITLE_NAME = "3D Visualizer";
const static char* CLOUD_NAME = "map points";

static CloudPtr generateTestCloud();
static void renderPointCloud(VisPtr viewer, CloudRGB cloud);
static VisPtr createVisualizer(string TITLE);
static CloudPtr MapPointsToCloudPtr(const vector<MapPoint>& points);
static void MapPointsToCloudRGB(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static void CamPosToCloudRGBVO(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static void CamPosToCloudRGBGT(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static void DepthToCloudRGB_VOPose(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static void DepthToCloudRGB_GTPose(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr);
static void filterPointCloud(CloudRGB & cloud, CloudRGB & cloud_filtered);
static void removeOutliers(CloudRGB & cloud, CloudRGB & cloud_filtered);
static void downSample(CloudRGB & cloud, CloudRGB & cloud_filtered);
static PointXYZRGB addPt(const Mat & t, PointXYZRGB basic_point, int r, int g, int b);
static PolygonMesh PossionReconstruction(CloudPtr cloud);
void printMatrix(const Mat & M, std::string matrix);
void RtToWorldT(const Mat & Rt, Mat & t_res);
void RtToWorldRT(const Mat & Rt, Mat & Rt_res);
void WorldRtToRT(const Mat & Rt, Mat & Rt_res);
static void meshReconstruction(VisPtr viewer, CloudRGB & cloud);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

#ifdef DEBUG_POINTCLOUD_VISUALIZER
static CloudPtr cloud;
#endif


////////////////////////////////// Initialization /////////////////////////////////
void PointCloudVisualizer::init()
{
#ifdef DEBUG_POINTCLOUD_VISUALIZER
    cloud = generateTestCloud();
#endif
    viewer = createVisualizer("3D Visualizer: Point Cloud");
    // viewer ->addCoordinateSystem (1);
#ifdef ShowMeshReconstruction
    viewer2 = createVisualizer("3D Visualizer: 3D Triangle Mesh");
    viewer2->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
#endif
}

void PointCloudVisualizer::process(DataManager& data, int frameIdx)
{
    cout << endl<< "last frame idx:" << frameIdx-1 <<endl;
    CloudRGB cloudMapPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudRGB cloudCamTrajectoryVO(new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudRGB cloudCamTrajectoryGT(new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudRGB cloudDepthPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // make sure that frameIdx can acutally associate to a frame
    if (frameIdx >= data.frames.size()) {
        frameIdx = data.frames.size() - 1;
    }

    #ifndef OnlyTrajectory
    #ifdef ShowOrbSlam
    if (data.mapPoints.size()>0)
        MapPointsToCloudRGB(viewer, data, frameIdx, cloudMapPoints);
    if (!data.frames[frameIdx].depthBuffer.empty())
        DepthToCloudRGB_VOPose(viewer, data, frameIdx, cloudDepthPoints);
    #ifdef ShowCameraTrajectory
    if (data.frames[frameIdx].Rt.rows)
        CamPosToCloudRGBVO(viewer, data, frameIdx, cloudCamTrajectoryVO);        // without ground truth R|t
    #endif
    #endif
    #endif


    #ifndef OnlyTrajectory
    #ifdef ShowGroundTruth
    if (data.frames[frameIdx].depthBuffer.rows)
        DepthToCloudRGB_GTPose(viewer, data, frameIdx, cloudMapPoints);            // with ground truth depth map
    #ifdef ShowCameraTrajectory
    if (data.frames[frameIdx].RtGt.rows)
        CamPosToCloudRGBGT(viewer, data, frameIdx, cloudCamTrajectoryGT);     // with ground truth R|t
    #endif
    #endif
    #endif


    // Temple dataset
    #ifdef OnlyTrajectory
    #ifdef ShowOrbSlam
    if (data.frames[frameIdx].Rt.rows)
        CamPosToCloudRGBVO(viewer, data, frameIdx, cloudCamTrajectoryVO);        // without ground truth R|t
    #endif
    #ifdef ShowGroundTruth
    if (data.frames[frameIdx].RtGt.rows)
        CamPosToCloudRGBGT(viewer, data, frameIdx, cloudCamTrajectoryGT);     // with ground truth R|t
    #endif
    #endif


    #ifdef ShowMeshReconstruction
    // TODO:: Mesh reconstruction from point clouds
    if (cloudMapPoints->points.size()>20)
    {
        //pcl::PolygonMesh triangles = PossionReconstruction(cloud);
        //viewer->addPolygonMesh(triangles, "polygon", 0);
        meshReconstruction(viewer2, cloudMapPoints);
    }
    #endif

    viewer->spinOnce (100); boost::this_thread::sleep
            (boost::posix_time::microseconds (100));
}

bool PointCloudVisualizer::validationCheck(DataManager& data, int frameIdx)
{
    return true;
}

static VisPtr createVisualizer(string TITLE)
{
    VisPtr viewer = VisPtr(new pcl::visualization::PCLVisualizer (TITLE));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
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


////////////////////////////////// Draw trajectory /////////////////////////////////
void printMatrix(const Mat &M, std::string matrix)
{
    printf("Matrix \"%s\" is %i x %i\n", matrix.c_str(), M.rows, M.cols);
    std::cout << M << std::endl;
}

void RtToWorldT(const Mat &Rt, Mat &t_res)
{
    Mat R = Mat::zeros(3,3, CV_64F);
    Mat t = Mat::zeros(3,1, CV_64F);
    t_res = Mat::zeros(3,1, CV_64F);
    Rt(Range(0,3),Range(0,3)).copyTo(R);
    Rt(Range(0,3),Range(3,4)).copyTo(t);
    Mat t_new = -R.inv()*t;
    t_new.copyTo(t_res);
}

void RtToWorldRT(const Mat& Rt, Mat &Rt_res)
{
    Mat R = Mat::zeros(3,3, CV_64F);
    Mat t = Mat::zeros(3,1, CV_64F);
    Rt_res = Mat::zeros(3,4, CV_64F);
    Rt(Range(0,3),Range(0,3)).copyTo(R);
    Rt(Range(0,3),Range(3,4)).copyTo(t);
    Mat t_new = -R.inv()*t;
    R.copyTo(Rt_res(Range(0,3),Range(0,3)));
    t_new.copyTo(Rt_res(Range(0,3),Range(3,4)));
}

void WorldRtToRT(const Mat& Rt, Mat &Rt_res)
{
    Mat R = Mat::zeros(3,3, CV_64F);
    Mat t = Mat::zeros(3,1, CV_64F);
    Rt_res = Mat::zeros(3,4, CV_64F);
    Rt(Range(0,3),Range(0,3)).copyTo(R);
    Rt(Range(0,3),Range(3,4)).copyTo(t);
    Mat t_new = -R*t;
    R.copyTo(Rt_res(Range(0,3),Range(0,3)));
    t_new.copyTo(Rt_res(Range(0,3),Range(3,4)));
}

void DrawCamera(VisPtr viewer, const Mat &Rt, int frameIdx, string name)
{
    double dist = 0.1;
    double scale = 0.3;
    Mat x = Mat::zeros(3,1,CV_64F); x.at<double>(0,0) = dist; 
    Mat y = Mat::zeros(3,1,CV_64F); y.at<double>(1,0) = dist; 
    Mat z = Mat::zeros(3,1,CV_64F); z.at<double>(2,0) = dist; 
    Mat R = Mat::eye(3,3,CV_64F);
    Mat t = Mat::zeros(3,1,CV_64F);
    // printMatrix(Rt, "Rt");
    // cout << frameIdx << endl;
    
    Rt(Range(0,3), Range(0,3)).copyTo(R);
    Rt(Range(0,3), Range(3,4)).copyTo(t);
   
    Mat pos = Mat::zeros(3,1,CV_64F);
    RtToWorldT(Rt, pos);
    PointXYZ curr_pos(pos.at<double>(0,0),pos.at<double>(1,0),pos.at<double>(2,0));
    
    Mat x1 = R.inv() * (x-t);
    Mat y1 = R.inv() * (y-t);
    Mat z1 = R.inv() * (z-t);

    PointXYZ x_axis(x1.at<double>(0,0),x1.at<double>(1,0),x1.at<double>(2,0));
    PointXYZ y_axis(y1.at<double>(0,0),y1.at<double>(1,0),y1.at<double>(2,0));
    PointXYZ z_axis(z1.at<double>(0,0),z1.at<double>(1,0),z1.at<double>(2,0));

    Mat dx = x1-pos;
    Mat dy = y1-pos;
    Mat dz = z1-pos;
    
    // x,y,z axis
    viewer->addLine(curr_pos, x_axis, 255, 0, 0,  "axis_x"+name+to_string(frameIdx), 0);
    viewer->addLine(curr_pos, y_axis, 0, 255, 0,  "axis_y"+name+to_string(frameIdx), 0);
    viewer->addLine(curr_pos, z_axis, 0, 0, 255,  "axis_z"+name+to_string(frameIdx), 0);
    // printMatrix(t, "curr_pos");
    // printMatrix(x1, "x_axis");
    // printMatrix(y1, "y_axis");
    // printMatrix(z1, "z_axis");

    #ifdef drawCameraPyramid
    // draw camera pyramid
    Mat rect1 = ( dx + dy)*scale + pos;
    Mat rect2 = ( dx - dy)*scale + pos;
    Mat rect3 = (-dx - dy)*scale + pos;
    Mat rect4 = (-dx + dy)*scale + pos;
    PointXYZ l1(rect1.at<double>(0,0),rect1.at<double>(1,0),rect1.at<double>(2,0));
    PointXYZ l2(rect2.at<double>(0,0),rect2.at<double>(1,0),rect2.at<double>(2,0));
    PointXYZ l3(rect3.at<double>(0,0),rect3.at<double>(1,0),rect3.at<double>(2,0));
    PointXYZ l4(rect4.at<double>(0,0),rect4.at<double>(1,0),rect4.at<double>(2,0));
    viewer->addLine(l1, l2, 0, 255, 0,  "l1"+name+to_string(frameIdx), 0);
    viewer->addLine(l2, l3, 0, 255, 0,  "l2"+name+to_string(frameIdx), 0);
    viewer->addLine(l3, l4, 0, 255, 0,  "l3"+name+to_string(frameIdx), 0);
    viewer->addLine(l4, l1, 0, 255, 0,  "l4"+name+to_string(frameIdx), 0);

    // square pyramid lines
    // viewer->addLine(l1, z_axis, 0, 255, 0,  "z1"+name+to_string(frameIdx), 0);
    // viewer->addLine(l2, z_axis, 0, 255, 0,  "z2"+name+to_string(frameIdx), 0);
    // viewer->addLine(l3, z_axis, 0, 255, 0,  "z3"+name+to_string(frameIdx), 0);
    // viewer->addLine(l4, z_axis, 0, 255, 0,  "z4"+name+to_string(frameIdx), 0);
    #endif
}


////////////////////////////////// Draw point cloud /////////////////////////////////
static PointXYZRGB addPt(const Mat &t, PointXYZRGB basic_point, int r, int g, int b)
{
    basic_point.x = t.at<double>(0,0);
    basic_point.y = t.at<double>(1,0);
    basic_point.z = t.at<double>(2,0);
    basic_point.r = 220;
    basic_point.g = 30;
    basic_point.b = 30;
    return basic_point;
}

static void CamPosToCloudRGBVO(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr)
{
    pcl::PointXYZRGB basic_point(0,0,0), pre_point(0,0,0);
    Mat Rt = Mat::zeros(3,4, CV_64F);   
    Mat t = Mat::zeros(3,1, CV_64F);    
    Mat Rt_world = Mat::zeros(3,4, CV_64F);    
    if (frameIdx > 0){
        for (int i=frameIdx-1; i<=frameIdx; i++)
        {
            pre_point.x = basic_point.x;
            pre_point.y = basic_point.y;
            pre_point.z = basic_point.z;
            (data.frames[i].Rt).copyTo(Rt);
            Rt(Range(0,3), Range(3,4)).copyTo(t);
            RtToWorldT(Rt,t);
            basic_point = addPt(t, basic_point, 220, 30, 30);
        }
        // cout << pre_point.x<<" "<<pre_point.y<<" "<<pre_point.z<<endl;
        // cout << basic_point.x<<" "<<basic_point.y<<" "<<basic_point.z<<endl;
        viewer->addLine(pre_point, basic_point, 250, 20, 20, to_string(frameIdx), 0);
    }
    
    (data.frames[frameIdx].Rt).copyTo(Rt_world);
    // RtToWorldRT(data.frames[frameIdx].Rt, Rt_world);
    DrawCamera(viewer, Rt_world, frameIdx, "VO");
}

static void CamPosToCloudRGBGT(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & basic_cloud_ptr)
{
    pcl::PointXYZRGB basic_point_gt(0,0,0), pre_point_gt(0,0,0);
    Mat Rt = Mat::zeros(3,4, CV_64F);
    Mat t = Mat::zeros(3,1, CV_64F);
    Mat Rt_world = Mat::zeros(3,4, CV_64F);
    if (frameIdx > 0) {
        for (int i=frameIdx-1; i<=frameIdx; i++)
        {
            pre_point_gt.x = basic_point_gt.x;
            pre_point_gt.y = basic_point_gt.y;
            pre_point_gt.z = basic_point_gt.z;
            (data.frames[i].RtGt).copyTo(Rt);
            Rt(Range(0,3), Range(3,4)).copyTo(t);
            RtToWorldT(Rt,t);
            basic_point_gt = addPt(t, basic_point_gt, 30, 220, 30);
        }
        // cout << pre_point_gt.x<<" "<<pre_point_gt.y<<" "<<pre_point_gt.z<<endl;
        // cout << basic_point_gt.x<<" "<<basic_point_gt.y<<" "<<basic_point_gt.z<<endl;
        viewer->addLine(pre_point_gt, basic_point_gt, 20, 250, 20, "gt"+to_string(frameIdx), 0);
    }
    // cout<<frameIdx-1 << ")\tCamera est. pos: \t"<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z;
    // cout<< "\tVS\tGT: \t"<<basic_point_gt.x<<","<<basic_point_gt.y<<","<<basic_point_gt.z<<endl;
    
    (data.frames[frameIdx].RtGt).copyTo(Rt_world);
    DrawCamera(viewer, Rt_world, frameIdx, "GT");
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


vector<int> getPixelRGBAvg(DataManager & data, int i)
{
    vector<int> rgb;
    MapPoint &p=data.mapPoints[i];
    rgb.push_back(0);
    rgb.push_back(0);
    rgb.push_back(0);
    map<int, int> observerToIndex = p.observerToIndex;
    int rsum=0;
    for (map<int,int>::iterator i = observerToIndex.begin(); i != observerToIndex.end(); ++i)
    {   
        Frame *this_frame = Util::findFrameById(data.frames, i->first); 
        
        if (this_frame != NULL && this_frame->features.positions.size()>=i->second){
            double x = this_frame->features.positions[i->second].x;
            double y = this_frame->features.positions[i->second].y;
            double b = this_frame->frameBuffer.at<Vec3b>(y,x)(0);
            double g = this_frame->frameBuffer.at<Vec3b>(y,x)(1);
            double r = this_frame->frameBuffer.at<Vec3b>(y,x)(2);
            rgb[0]+=r;
            rgb[1]+=g;
            rgb[2]+=b;
            //cout<<" color "<<endl;
        }else{
            // #undef displayRGBPointCloud
        }
    }
    rgb[0]/=observerToIndex.size();
    rgb[1]/=observerToIndex.size();
    rgb[2]/=observerToIndex.size();
    return rgb;
}


static void MapPointsToCloudRGB(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB &basic_cloud_ptr)
{
    for (int i=0; i<data.mapPoints.size(); i++)
    {
        const MapPoint &point = data.mapPoints[i];
        pcl::PointXYZRGB basic_point;
        basic_point.x = point.worldPosition.x;
        basic_point.y = point.worldPosition.y;
        basic_point.z = point.worldPosition.z;
        vector<int> rgb = getPixelRGBAvg(data, i); 
        basic_point.r = rgb[0]; 
        basic_point.g = rgb[1];  
        basic_point.b = rgb[2];
        // cout<<"pos: "<<basic_point.x<<","<<basic_point.y<<","<<basic_point.z<<endl;
        basic_cloud_ptr->points.push_back(basic_point);
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    
    if (viewer->contains(CLOUD_NAME)) {
        viewer->updatePointCloud(basic_cloud_ptr, CLOUD_NAME);
    }
    else {
        viewer->addPointCloud(basic_cloud_ptr, CLOUD_NAME);
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, CLOUD_NAME);
}


////////////////////////////////// Depth map visualization /////////////////////////////////
static void DepthToCloudRGB_VOPose(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & cloudMapPoints)
{
    Mat depImg = data.frames[frameIdx].depthBuffer;
    Mat rgbImg = data.frames[frameIdx].frameBuffer;
    int h = depImg.rows;
    int w = depImg.cols;
    int step = 5;
    double fx = data.camera_intrinsics.at<double>(0,0);
    double fy = data.camera_intrinsics.at<double>(1,1);
    double cx = data.camera_intrinsics.at<double>(0,2);
    double cy = data.camera_intrinsics.at<double>(1,2);
    float factor = 5000.0f;
    
    Mat Rt = Mat::zeros(3,4,CV_64F);
    (data.frames[frameIdx].Rt).copyTo(Rt);
    Mat R = Mat::zeros(3,3,CV_64F);
    Mat t = Mat::zeros(3,1,CV_64F);
    Rt(Range(0,3), Range(0,3)).copyTo(R);
    Rt(Range(0,3), Range(3,4)).copyTo(t);
    // RtToWorldT(Rt, t);

    for (int i=0; i<h; i+=step)
    {
        for (int j=0; j<w; j+=step)
        {
            Mat loc = Mat::zeros(3,1,CV_64F);
            loc.at<double>(2,0) = depImg.at<float>(i,j)/factor;
            loc.at<double>(0,0) = (i-cx) * loc.at<double>(2,0) / fx;
            loc.at<double>(1,0) = (j-cy) * loc.at<double>(2,0) / fy;
            loc = R.inv()*(loc-t);

            pcl::PointXYZRGB pixel;
            pixel.x = loc.at<double>(0,0);
            pixel.y = loc.at<double>(1,0);
            pixel.z = loc.at<double>(2,0);
            pixel.b = rgbImg.data[i * rgbImg.step + 3 * j ];
            pixel.g = rgbImg.data[i * rgbImg.step + 3 * j + 1];
            pixel.r = rgbImg.data[i * rgbImg.step + 3 * j + 2];

            cloudMapPoints->points.push_back(pixel);
        }
    }
    cloudMapPoints->width = (int) cloudMapPoints->points.size();

    #ifdef PlotAllFrames
    string DEPTHMAP_NAME_ALL = "depth map_vo"+to_string(frameIdx);
    viewer->addPointCloud(cloudMapPoints, DEPTHMAP_NAME_ALL);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, DEPTHMAP_NAME_ALL);
    #else
    string DEPTHMAP_NAME = "depth map_vo";
    if (viewer->contains(DEPTHMAP_NAME)) {
        viewer->updatePointCloud(cloudMapPoints, DEPTHMAP_NAME);
    }
    else {
        viewer->addPointCloud(cloudMapPoints, DEPTHMAP_NAME);
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, DEPTHMAP_NAME);
    #endif

    // cout <<"Depth cloud : # of pts: "<<cloudMapPoints->points.size()<<endl;
}

static void DepthToCloudRGB_GTPose(VisPtr viewer, DataManager& data, int frameIdx, CloudRGB & cloudMapPoints)
{
    Mat depImg = data.frames[frameIdx].depthBuffer;
    Mat rgbImg = data.frames[frameIdx].frameBuffer;
    int h = depImg.rows;
    int w = depImg.cols;
    double fx = data.camera_intrinsics.at<double>(0,0);
    double fy = data.camera_intrinsics.at<double>(1,1);
    double cx = data.camera_intrinsics.at<double>(0,2);
    double cy = data.camera_intrinsics.at<double>(1,2);
    float factor = 5000.0f;
    Mat Rt = Mat::zeros(3,4,CV_64F);
    (data.frames[frameIdx].RtGt).copyTo(Rt);
    Mat R  = Mat::zeros(3,3,CV_64F);
    Mat t  = Mat::zeros(3,1,CV_64F);
    Rt(Range(0,3), Range(0,3)).copyTo(R); 
    Rt(Range(0,3), Range(3,4)).copyTo(t); 
    int step = 1.0/depth_density_ratio;
    for (int i=0; i<h; i+=step)
    {
        for (int j=0; j<w; j+=step)
        {
            Mat loc = Mat::zeros(3,1,CV_64F);
            loc.at<double>(2,0) = depImg.at<float>(i,j)/factor;
            loc.at<double>(0,0) = (i-cx) * loc.at<double>(2,0) / fx;
            loc.at<double>(1,0) = (j-cy) * loc.at<double>(2,0) / fy;
            loc = R.inv()*(loc-t);

            pcl::PointXYZRGB pixel;
            pixel.x = loc.at<double>(0,0);
            pixel.y = loc.at<double>(1,0);
            pixel.z = loc.at<double>(2,0);
            pixel.b = rgbImg.data[i * rgbImg.step + 3 * j ];
            pixel.g = rgbImg.data[i * rgbImg.step + 3 * j + 1];
            pixel.r = rgbImg.data[i * rgbImg.step + 3 * j + 2];

            cloudMapPoints->points.push_back(pixel);
        }
    }
    cloudMapPoints->width = (int) cloudMapPoints->points.size ();

    #ifdef PlotAllFrames
    string DEPTHMAP_NAME_ALL = "depth map_gt"+to_string(frameIdx);
    viewer->addPointCloud(cloudMapPoints, DEPTHMAP_NAME_ALL);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, DEPTHMAP_NAME_ALL);
    #else
    string DEPTHMAP_NAME = "depth map_gt";
    if (viewer->contains(DEPTHMAP_NAME)) {
        viewer->updatePointCloud(cloudMapPoints, DEPTHMAP_NAME);
    }
    else {
        viewer->addPointCloud(cloudMapPoints, DEPTHMAP_NAME);
    }
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, DEPTHMAP_NAME);
    #endif
}


/////////////////////////// Mesh reconstruction ///////////////////////////
// Mesh possion reconstruction for CloudPtr
static PolygonMesh PossionReconstruction(CloudPtr& cloud)
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

static void filterPointCloud(CloudRGB & cloud, CloudRGB & cloud_filtered)
{
    pcl::PassThrough<PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName ("x");
    filter.setFilterLimits (-4000, 4000);
    filter.setFilterFieldName ("y");
    filter.setFilterLimits (-4000, 4000);
    filter.setFilterFieldName ("z");
    filter.setFilterLimits (-4000, 4000);
    filter.filter(*cloud_filtered);
    cout << "[ "<<cloud_filtered->width << " points ] after PassThrough filter.  "  << endl;
}

static void downSample(CloudRGB & cloud, CloudRGB & cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);          // created with a leaf size of 1cm
    sor.setLeafSize(0.01f, 0.01f, 0.01f); 
    sor.filter(*cloud_filtered);
    cout << "[ " << cloud_filtered->width << " points ] after downsampling by VoxelGrid. " <<endl;
}

static void removeOutliers(CloudRGB & cloud, CloudRGB & cloud_filtered)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    // Set number of neighbors to analyze
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    //sor.setNegative (true);  // to get outliers only
    sor.filter (*cloud_filtered);
    cout  << "[ " << cloud_filtered->width << " points ] after StatisticalOutlierRemoval.  " << endl;
}

// Mesh possion reconstruction for CloudRGB
static void meshReconstruction(VisPtr viewer2, CloudRGB &cloud_ori)
{
    int pts_num = cloud_ori->points.size();
    cout << "Started Poisson mesh reconstruction [ "<<pts_num <<" points ] " <<endl;

    // Filtered by PassThrough
    CloudRGB filtered(new pcl::PointCloud<PointXYZRGB>());
    filterPointCloud(cloud_ori,filtered);

    // Downsampled by VoxelGrid
    CloudRGB sampled(new pcl::PointCloud<PointXYZRGB>());
    //downSample(cloud_ori, sampled);
    downSample(filtered, sampled);

    // Filtered by StatisticalOutlierRemoval
    CloudRGB filtered2(new pcl::PointCloud<PointXYZRGB>());
    //removeOutliers(cloud_ori, filtered2);
    removeOutliers(sampled, filtered2);
    
    CloudRGB cloud = filtered2;

    // Normal estimation
    pcl::NormalEstimation<PointType, Normal> normEst;
    pcl::PointCloud<Normal>::Ptr normals (new pcl::PointCloud<Normal>);

    // Create kdtree representation of cloud,
    // and pass it to the normal estimation object.
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (cloud);
    normEst.setInputCloud (cloud);
    normEst.setSearchMethod (tree);

    // Use neighbor points for estimating normal
    normEst.setKSearch (MIN(pts_num, 20));
    normEst.compute (*normals);
    // normals should not contain the point normals + surface
    // curvatures

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals

    // Create search tree
    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    // psn - for surface reconstruction algorithm
    // triangles - for storage of reconstructed triangles
    pcl::Poisson<PointTypeN> psn;
    pcl::PolygonMesh triangles;

    psn.setInputCloud(cloud_with_normals);
    psn.setSearchMethod(tree2);
    psn.setDepth(8);                            // psn_depth
    psn.setSolverDivide(8);                     // setSolverDivide
    psn.setIsoDivide(8);                        // isoDivide
    psn.setSamplesPerNode(1);                    // psn_samplesPerNode
    psn.setScale(1.1);
    psn.setConfidence(false);
    psn.reconstruct (triangles);

    //psn.setOutputPolygons(false);
    //std::string str, str2;
    //str.append(filePath).append("-mesh.vtk");
    //pcl::io::saveVTKFile (str, triangles);
    
    // Create viewer object and show mesh
    string meshname = "3D mesh";
    if (viewer2->contains(meshname)) {
        viewer2->updatePolygonMesh(triangles, meshname);
    }
    else {
        viewer2->addPolygonMesh(triangles, meshname);
    }
    
    // Setting type of mesh representation
    // Wireframe = standard "mesh" representation
    switch(displayForm){
        case 0: 
            viewer2->setRepresentationToWireframeForAllActors ();
            break;
        case 1:
            viewer2->setRepresentationToSurfaceForAllActors ();
            break;
        default:
            viewer2->setRepresentationToPointsForAllActors ();
            break;
    }

    viewer2->spinOnce (1000); boost::this_thread::sleep
            (boost::posix_time::microseconds (10000));

}


////////////////////////////////// keyborad response ////////////////////////////
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer2 = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "m" && event.keyDown ())
    {
        std::cout << "Change mesh presentation. " << std::endl;
        displayForm = (displayForm+1)%3;
    }
}