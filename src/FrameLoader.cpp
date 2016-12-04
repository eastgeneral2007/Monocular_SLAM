//
// FrameLoader.cpp
//
// Frame loader loaded all the frames from the directory
// and feed them into frames.
//
// @Yu, Sharon

#include "DataManager.h"
#include "FrameLoader.h"
#include <fstream>
#include <string.h>
#include <iostream>
using namespace std;

bool has_suffix(const string& s, const string& suffix)
{
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

Frame parseImgRawFileName(string directory, string filename)
{
    double timestamp;
    sscanf(filename.c_str(), "%lf.png", &timestamp);
    Frame f; 
    FrameMeta& meta = f.meta;
    meta.timestamp = timestamp;
    meta.framename = directory + filename;
    return f;
}

void loadImgFileList(string directory, int begin_frame, int end_frame, DataManager& data, int step) {
    if (directory.back()!='/') {
        directory += '/';
    }

    DIR *pDIR;
    int index=0;
    struct dirent *entry;
    vector<Frame> imgNamelist;
    if( (pDIR=opendir(directory.c_str())) !=NULL){
        while((entry = readdir(pDIR))!= NULL){
            if( has_suffix(entry->d_name, ".png") || has_suffix(entry->d_name, ".jpg") ) {
                imgNamelist.push_back(parseImgRawFileName(directory, entry->d_name));
                index++;
            }
        }
        closedir(pDIR);
    }
    sort(imgNamelist.begin(), imgNamelist.end());

    for (int i=begin_frame; i<MIN(end_frame, imgNamelist.size()); i+=step){
        Frame f;
        FrameMeta& meta = f.meta;
        meta.timestamp=imgNamelist[i].meta.timestamp;
        meta.framename=imgNamelist[i].meta.framename;
        meta.frameID=i;
        f.frameBuffer=imread(imgNamelist[i].meta.framename, CV_LOAD_IMAGE_UNCHANGED);
        // imshow("frame", f.frameBuffer);
        // waitKey(40);
        data.frames.push_back(f);
    }
}

Mat loadRT(double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
    Mat Rt = Mat::zeros(3,4,CV_64F);
    // q->R
    Rt.at<double>(0,0) = 1 - 2*qy*qy - 2*qz*qz;
    Rt.at<double>(0,1) = 2*qx*qy - 2*qz*qw;
    Rt.at<double>(0,2) = 2*qx*qz + 2*qy*qw;
    Rt.at<double>(1,0) = 2*qx*qy + 2*qz*qw;
    Rt.at<double>(1,1) = 1 - 2*qx*qx - 2*qz*qz;
    Rt.at<double>(1,2) = 2*qy*qz - 2*qx*qw;
    Rt.at<double>(2,0) = 2*qx*qz - 2*qy*qw;
    Rt.at<double>(2,1) = 2*qy*qz + 2*qx*qw;
    Rt.at<double>(2,2) = 1 - 2*qx*qx - 2*qy*qy;
    // t'->t
    Rt.at<double>(0,3) = tx;
    Rt.at<double>(1,3) = ty;
    Rt.at<double>(2,3) = tz;
    return Rt.clone();
}

void loadGroundTruth(string filename, int begin_frame, int end_frame, DataManager& data) {
    assert(data.frames.size()>=1);

    if (filename.back()=='/') {
        filename.pop_back();
    }
    double tx, ty, tz, qx, qy, qz, qw; 
    double pre_tx = 0, pre_ty = 0, pre_tz = 0;
    ifstream fin (filename);
    double time_stamp = data.frames[0].meta.timestamp;
    int frameIdx = 0;
    double time_stamp_gt;
    if (fin.is_open())
    {
        char str[256];
        fin.getline(str, 256);
        fin.getline(str, 256);
        fin.getline(str, 256);
        fin.getline(str, 256);
        sscanf(str, "%lf %lf %lf %lf %lf %lf %lf %lf", &time_stamp_gt, &tx, &ty, &tz, &qx, &qy, &qz, &qw);

        while (!fin.eof() && frameIdx < data.frames.size()-1)
        {
            while (time_stamp >= time_stamp_gt+0.02)
            {
                pre_tx = tx + pre_tx;
                pre_ty = ty + pre_ty;
                pre_tz = tz + pre_tz;
                fin.getline(str, 256);
                sscanf(str, "%lf %lf %lf %lf %lf %lf %lf %lf", &time_stamp_gt, &tx, &ty, &tz, &qx, &qy, &qz, &qw);
            }
            //data.frames[frameIdx].RtGt = loadRT(pre_tx, pre_ty, pre_tz, qx, qy, qz, qw);
            data.frames[frameIdx].RtGt = loadRT(tx, ty, tz, qx, qy, qz, qw);
            //printf("(%d) %lf VS %lf :  ", frameIdx, time_stamp, time_stamp_gt);
            //cout << data.frames[frameIdx].RtGt.at<double>(0,3) <<", "<<
            //        data.frames[frameIdx].RtGt.at<double>(1,3) <<", "<<
            //        data.frames[frameIdx].RtGt.at<double>(2,3) <<"    VS"  <<
            //        tx << ", "<<ty <<", " <<tz << endl;
            time_stamp = data.frames[++frameIdx].meta.timestamp;
        }
        fin.close();
    }else{
        cout << "Can't find ground truth file "+filename << endl;
        return;
    }    
}

// Original dataset
void loadCameraIntrinsics_TUM1(DataManager& data) {
    // TODO: implement a proper loader in the future
    static double fx = 517.306408;
    static double fy = 516.469215;
    static double cx = 318.643040;
    static double cy = 255.313989;

    Mat& camera_intrinsics = data.camera_intrinsics;
    camera_intrinsics = Mat::zeros(3, 3, CV_64F);
    camera_intrinsics.at<double>(0,0) = fx;
    camera_intrinsics.at<double>(0,2) = cx;
    camera_intrinsics.at<double>(1,1) = fy;
    camera_intrinsics.at<double>(1,2) = cy;
    camera_intrinsics.at<double>(2,2) = 1.f;
}

// TUM f1/f3
void loadCameraIntrinsics_kinect(DataManager& data) {
    // TODO: implement a proper loader in the future
    static double fx = 525.0;
    static double fy = 525.0;
    static double cx = 319.5;
    static double cy = 239.5;

    Mat& camera_intrinsics = data.camera_intrinsics;
    camera_intrinsics = Mat::zeros(3, 3, CV_64F);
    camera_intrinsics.at<double>(0,0) = fx;
    camera_intrinsics.at<double>(0,2) = cx;
    camera_intrinsics.at<double>(1,1) = fy;
    camera_intrinsics.at<double>(1,2) = cy;
    camera_intrinsics.at<double>(2,2) = 1.0f;
}

void FrameLoader::load(DataManager& data) {
    if (directory.find("rgbd_dataset_freiburg1_desk2_secret")!=string::npos)
    {
        loadCameraIntrinsics_TUM1(data);
        loadImgFileList(directory, begin_frame, end_frame, data, step);
    }else{
        loadCameraIntrinsics_kinect(data);
        loadImgFileList(directory+"/rgb/", begin_frame, end_frame, data, step);
        loadGroundTruth(directory+"/groundtruth.txt/", begin_frame, end_frame, data);
    }
}
