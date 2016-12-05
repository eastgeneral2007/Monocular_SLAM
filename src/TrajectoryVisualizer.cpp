//
// TrajectoryVisualizer.h
//
// Trajectory Visualization 
//
// @Sharon


#include "TrajectoryVisualizer.h"
using namespace cv;
extern void printMatrix(const Mat & M, std::string matrix);
extern void RtToWorldT(const Mat & Rt, Mat & t_res);

// Draw text on image
void drawState(Mat & img, string msg, int row = 1) {
    Point org(30, MIN(img.rows, 330 + 30 * row));
    int fontFace = CV_FONT_NORMAL;
    double fontScale = 0.5;
    int thickness = 1;
    Scalar color(250, 30, 30);
    putText(img, msg.c_str(), org, fontFace, fontScale, color, thickness);
}

// Draw odometry map on image
void drawOdometryMap(DataManager& data, int frameIdx, int size_p, double map_range, int row = 1) {
    const Scalar background_color(0,0,50);
    const Scalar loc_color(0,0,255);
    const Scalar trace_color(0,255,0);
    const int p_radius = 2;

    const double pixel_per_meter = double(size_p)/map_range;
    Mat img = data.frames[frameIdx].frameBuffer;
    const int base_x = img.cols-size_p;
    const int base_y = img.rows-size_p * row;
    const Point ref_center(size_p/2+base_x, size_p/2+base_y);
    // cout << "roi:  "<< base_x << " " << base_y << " " << size_p << " " << size_p << endl;
    
    Mat roi(img, Rect(base_x, base_y, size_p, size_p));
    roi = background_color;
    
    Mat t = Mat::zeros(3,1, CV_64F);
    Mat t_pre = Mat::zeros(3,1, CV_64F);
    Mat Rt = Mat::zeros(3,4, CV_64F);
    Mat Rt_pre = Mat::zeros(3,4, CV_64F);
    // (data.frames[frameIdx].Rt).copyTo(Rt);
    // (data.frames[frameIdx-1].Rt).copyTo(Rt_pre);
    // RtToWorldT(Rt, t);
    // RtToWorldT(Rt_pre, t_pre);
    (data.frames[frameIdx].RtGt).copyTo(Rt);
    (data.frames[frameIdx-1].RtGt).copyTo(Rt_pre);
    RtToWorldT(Rt, t);
    RtToWorldT(Rt_pre, t_pre);
    
    Point3f curr_loc( t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));
    Point3f prev_loc( t_pre.at<double>(0,0), t_pre.at<double>(1,0), t_pre.at<double>(2,0));

    /*
    for (int i = 0; i <= frameIdx; ++i) {
        // double dx = data.frames[i].Rt.at<double>(0,3) - curr_loc.x;
        // double dz = data.frames[i].Rt.at<double>(2,3) - curr_loc.z;
        double dx = data.frames[i].RtGt.at<double>(0,3) - curr_loc.x;
        double dz = data.frames[i].RtGt.at<double>(1,3) - curr_loc.y;
        if (abs(dx) < map_range/2 && abs(dz) < map_range/2)
        {
            Point center;
            center.x = ref_center.x + dx * pixel_per_meter;
            center.y = ref_center.y - dz * pixel_per_meter;

            circle( img, center, p_radius, trace_color, -1);
        }
    }
    */

    circle( img, ref_center, p_radius+1, loc_color, -1);

    // Mat curr_pose = data.frames[frameIdx].Rt;
    Mat curr_pose = data.frames[frameIdx].RtGt;
    Mat forward = Mat::zeros(4,1,curr_pose.type());
    forward.at<double>(2) = map_range/3;
    forward.at<double>(3) = 1;
    Mat r = curr_pose * forward;

    // Point3f vec(r.at<double>(2), r.at<double>(1), -r.at<double>(0));
    Point3f vec(r.at<double>(0), r.at<double>(1), -r.at<double>(2));
    Point target;
    target.x = MAX(MIN(ref_center.x + (vec.x - curr_loc.x)*pixel_per_meter, size_p+base_x),base_x);
    target.y = MAX(MIN(ref_center.y + (vec.z - curr_loc.z)*pixel_per_meter, size_p+base_y),base_y);
    line(img, ref_center, target, Scalar(255,0,0),2);

    // note of scale in the left_bottom corner
    double scale = map_range/10;
    char str[200];
    sprintf(str, "%.1fm", (float)scale);
    putText(img, str, Point(base_x -5,  base_y + size_p - 10), CV_FONT_NORMAL, 0.4, Scalar(0, 255, 0), 1);//base_x+5,
    line( img,
            Point(base_x+5, base_y+ size_p - 5),
            Point(base_x+5+size_p/10, base_y+ size_p-5),
            Scalar( 0, 255, 0 ),
            1);
    
    // plot motion vector and text
    Point3f motion = curr_loc - prev_loc;
    double speed = norm(motion) / (data.frames[frameIdx].meta.timestamp-data.frames[frameIdx-1].meta.timestamp);
    double heading = atan2(motion.z, motion.x);
    
    sprintf(str, "Frame %i   Timestamp=%04.03lf", frameIdx-1, data.frames[frameIdx].meta.timestamp);
    drawState(img, str, 1);

    sprintf(str, "Speed = %5f m/s  Direction=%5lf", speed, heading);
    //drawState(img, str, 2);

    sprintf(str, "previously at (%.2lf, %.2lf, %.2lf)", prev_loc.x, prev_loc.y, prev_loc.z);
    drawState(img, str, 2);

    sprintf(str, "currently at  (%.2lf, %.2lf, %.2lf)", curr_loc.x, curr_loc.y, curr_loc.z);
    drawState(img, str, 3);

    imshow("1", img);

    char k = waitKey(0);
}


void TrajectoryVisualizer::process(DataManager& data, int frameIdx)
{
    if (frameIdx > 1){
        int h = data.frames[frameIdx].frameBuffer.rows;
        int w = data.frames[frameIdx].frameBuffer.cols;
        int row = 1;
        int size_p = MIN(MIN(150, h/2.0),w/3.0);
        double map_range = 10;
        //cout << "width = " << w << endl;
        //cout << "height = " << h << endl;
        drawOdometryMap(data, frameIdx, size_p, map_range, row);
    }
}


bool TrajectoryVisualizer::validationCheck(DataManager& data, int frameIdx)
{
    return true;
}