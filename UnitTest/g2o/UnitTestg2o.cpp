//
// Created by JimXing on 19/11/16.
//

#include <iostream>
#include "UnitTestg2o.h"
#include  <math.h>

using namespace std;
using namespace cv;

void UnitTestg2o::unitTestCvMatToG2oSE3() {
    cout<< "Test unitTestCvMatToG2oSE3"<< endl;
    Mat R = (Mat_<double>(3,3) <<     0.9506,  0.2940,  -0.0998,
    -0.2845,    0.9535,   0.0993,
    0.1244,   -0.0660,    0.9900);
    Mat T = (Mat_<double>(3,1) << 2, 3, 1);
    cout << "R = " << endl << " " << R << endl << endl;
    cout << "T = " << endl << " " << T << endl << endl;

    Mat pose = Mat::zeros(3,4, CV_64F);
    R.copyTo(pose.colRange(0,3).rowRange(0,3)); // exclusive range second parameter
//    pose.at<double>(3,3) = 1.0;
    T.copyTo(pose.colRange(3,4).rowRange(0,3));
    cout << "pose = " << endl << " " << pose << endl << endl;

    g2o::SE3Quat pose_se3 = Converter::cvMatToSE3Quat(pose);
    cout << "pose_se3 = "<< endl << pose_se3 << endl;
}

void UnitTestg2o::unitTestFullBA() {
    DataManager dm;
    cv::Mat intrinsics_tum1 = (Mat_<double>(3,3) << 517.306408,  0,   318.643040,
    0,   516.469215,   255.313989,
    0,   0,    1);
    dm.camera_intrinsics = intrinsics_tum1;
    vector<Frame> frames_before_full_BA = Util::loadFrames("data/UnitTest/Frames.csv");
    vector<MapPoint> map_points_before_full_BA = Util::loadMapPoints("data/UnitTest/MapPoints.csv");
    cout << "finished loading before frames, # read in:" << frames_before_full_BA.size() << endl;
    cout << "finished loading before map points, # read in:" << map_points_before_full_BA.size() << endl;
    dm.frames = frames_before_full_BA;
    dm.mapPoints = map_points_before_full_BA;
    Util::GlobalBundleAdjustemnt(dm, FULL_BA_ITER);
    vector<Frame> frames_after_full_BA = Util::loadFramesOnlyIdRt("data/UnitTest/FramesOptimised.csv");
    cout << "finished loading after frames, # read in:" << frames_after_full_BA.size() << endl;
    vector<MapPoint> map_point_after_full_BA = Util::loadMapPointsOnlyIdXYZ("data/UnitTest/MapPointsOptimised.csv");
    cout << "finished loading after map points, # read in" << map_point_after_full_BA.size() << endl;
    
    // assert the difference between dm.frames and frames_after_full_BA
    double max_diff = -1;
    double Rt_thresh = 1;
    for (int i =0;i< dm.frames.size();i++) {
        Frame * this_frame = &dm.frames[i];
        Frame * compare_frame = Util::findFrameById(frames_after_full_BA, this_frame->meta.frameID);
        double this_diff = differenceInRtL2(this_frame, compare_frame);
        assert(this_diff < Rt_thresh);
        if (this_diff > max_diff) {
            max_diff = this_diff;
        }
    }
    cout << "Max CV_COMP_BHATTACHARYYA distance of Rt between loaded and executed full BA frames: " << max_diff << endl;

    // assert the difference between dm.mapPoint and map_point_after_full_BA
    double map_point_thresh = 1;
    max_diff = -1;
    int max_diff_map_id = -1;
    for (int i =0;i< dm.mapPoints.size();i++) {
        MapPoint * this_map_point = &dm.mapPoints[i];
        MapPoint * compare_map_point = Util::findMapPointById(map_point_after_full_BA, this_map_point->id);
        double this_diff = differenceInXYZL2(this_map_point, compare_map_point);
        assert(this_diff < map_point_thresh);
        if (this_diff > max_diff) {
            max_diff = this_diff;
            max_diff_map_id = this_map_point->id;
        }
    }
    cout << "Max Euclidean distance between XYZ of loaded and full BA MapPoints: " << max_diff << endl;

}

double UnitTestg2o::differenceInRtL2(Frame *f1, Frame *f2) {
    cout << "f1->Rt:\n" << f1->Rt << endl;
    cout << "f2->Rt:\n" << f2->Rt << endl;
    // Mat mat1 = f1->Rt;
    // Mat mat2 = f2->Rt;
    // mat1.convertTo(mat1, CV_32F);
    // mat2.convertTo(mat2, CV_32F);
    // return cv::compareHist(mat1, mat2, CV_COMP_BHATTACHARYYA);
    Mat abs_diff;
    cv::absdiff(f1->Rt, f2->Rt, abs_diff);
    Mat square_diff = abs_diff.mul(abs_diff);
    return cv::sum(square_diff)[0];
}

double UnitTestg2o::differenceInXYZL2(MapPoint *m1, MapPoint *m2) {
    return (pow(m1->worldPosition.x - m2->worldPosition.x, 2) + 
    pow(m1->worldPosition.y - m2->worldPosition.y, 2) +
    pow(m1->worldPosition.z - m2->worldPosition.z, 2)); 
}




