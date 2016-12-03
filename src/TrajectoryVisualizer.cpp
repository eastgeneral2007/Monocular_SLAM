//
// TrajectoryVisualizer.h
//
// Trajectory Visualization 
//
// @Sharon


#include "TrajectoryVisualizer.h"
using namespace cv;

// Draw text on image
void drawState(Mat &img, string msg, int row = 1) {
    Point org(50, MIN(img.rows, 300 + 30 * row));
    int fontFace = CV_FONT_NORMAL;
    double fontScale = 0.5;
    int thickness = 1;
    Scalar color(250, 30, 30);
    putText(img, msg.c_str(), org, fontFace, fontScale, color, thickness);
}


// Draw odometry map on image
void drawOdometryMap(DataManager& data, int frameIdx, int size_p, float map_range, int row = 1) {
    const Scalar background_color(0,0,50);
    const Scalar loc_color(0,0,255);
    const Scalar trace_color(0,255,0);
    const int p_radius = 2;

    const float pixel_per_meter = float(size_p)/map_range;
    Mat img = data.frames[frameIdx].frameBuffer;
    const int base_x = img.cols-size_p;
    const int base_y = size_p * row;
    const Point ref_center(size_p/2+base_x, size_p/2+base_y);
    // cout << "roi:  "<< base_x << " " << base_y << " " << size_p << " " << size_p << endl;
    
    Mat roi(img, Rect(base_x, base_y, size_p, size_p));
    roi = background_color;

    Point3f curr_loc( data.frames[frameIdx].Rt.at<float>(0,3), 
                      data.frames[frameIdx].Rt.at<float>(1,3), 
                      data.frames[frameIdx].Rt.at<float>(2,3));
    Point3f prev_loc( data.frames[frameIdx-1].Rt.at<float>(0,3), 
                      data.frames[frameIdx-1].Rt.at<float>(1,3), 
                      data.frames[frameIdx-1].Rt.at<float>(2,3));

    for (int i = 0; i < frameIdx; ++i) {
        float dx = data.frames[i].Rt.at<float>(0,3) - curr_loc.x;
        float dz = data.frames[i].Rt.at<float>(2,3) - curr_loc.z;
        if (abs(dx) < map_range/2 && abs(dz) < map_range/2)
        {
            Point center;
            center.x = ref_center.x + dx * pixel_per_meter;
            center.y = ref_center.y - dz * pixel_per_meter;

            circle( img, center, p_radius, trace_color, -1);
        }
    }

    circle( img, ref_center, p_radius+1, loc_color, -1);

    Mat curr_pose = data.frames[frameIdx].Rt;
    Mat forward = Mat::zeros(4,1,curr_pose.type());
    forward.at<float>(2) = map_range/3;
    forward.at<float>(3) = 1;
    Mat r = curr_pose * forward;

    Point3f vec(r.at<float>(2), r.at<float>(1), -r.at<float>(0));
    Point target;
    target.x = ref_center.x + (vec.x - curr_loc.x)*pixel_per_meter;
    target.y = ref_center.y - (vec.z - curr_loc.z)*pixel_per_meter;
    line(img, ref_center, target, Scalar(255,0,0),2);

    // note of scale in the left_bottom corner
    int scale = map_range/100;
    char str[200];
    sprintf(str, "%.1fm", scale);
    putText(img, str, Point(base_x -5,  base_y + size_p - 10), CV_FONT_NORMAL, 0.4, Scalar(0, 255, 0), 1);//base_x+5,
    line( img,
            Point(base_x+5, base_y+ size_p - 5),
            Point(base_x+5+size_p/10, base_y+ size_p-5),
            Scalar( 0, 255, 0 ),
            1);
    
    // plot motion vector and text
    Point3f motion = curr_loc - prev_loc;
    float speed = norm(motion) / (data.frames[frameIdx].meta.timestamp-data.frames[frameIdx-1].meta.timestamp);
    float heading = atan2(motion.z, motion.x);
    
    sprintf(str, "Frame %i  Timestamp=%05f", frameIdx, data.frames[frameIdx].meta.timestamp);
    drawState(img, str, 1);

    sprintf(str, "Speed=%5f m/s  Direction=%5f", speed, heading);
    //drawState(img, str, 2);

    sprintf(str, "previous location = (%2f, %2f, %2f)", prev_loc.x, prev_loc.y, prev_loc.z);
    drawState(img, str, 2);

    sprintf(str, "current location  = (%2f, %2f, %2f)", curr_loc.x, curr_loc.y, curr_loc.z);
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
        int size_p = MIN(MIN(150, h/2.0),w/2.0);
        int map_range = 10;
        //cout << "width = " << w << endl;
        //cout << "height = " << h << endl;
        drawOdometryMap(data, frameIdx, size_p, map_range, row);
    }
}


bool TrajectoryVisualizer::validationCheck(DataManager& data, int frameIdx)
{
    return true;
}