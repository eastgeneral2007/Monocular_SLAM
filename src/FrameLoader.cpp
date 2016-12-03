//
// FrameLoader.cpp
//
// Frame loader loaded all the frames from the directory
// and feed them into frames.
//
// @Yu

#include "DataManager.h"
#include "FrameLoader.h"

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

void loadImgFileList(string directory, int begin_frame, int end_frame, DataManager& data) {
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

    for (int i=begin_frame; i<MIN(end_frame, imgNamelist.size()); i++){
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

// Original dataset
void loadCameraIntrinsics_TUM1(DataManager& data) {
    // TODO: implement a proper loader in the future
    static float fx = 517.306408;
    static float fy = 516.469215;
    static float cx = 318.643040;
    static float cy = 255.313989;

    Mat& camera_intrinsics = data.camera_intrinsics;
    camera_intrinsics = Mat::zeros(3, 3, CV_64F);
    camera_intrinsics.at<double>(0,0) = fx;
    camera_intrinsics.at<double>(0,2) = cx;
    camera_intrinsics.at<double>(1,1) = fy;
    camera_intrinsics.at<double>(1,2) = cy;
    camera_intrinsics.at<double>(2,2) = 1.f;
}

// TUM360
void loadCameraIntrinsics_kinect(DataManager& data) {
    // TODO: implement a proper loader in the future
    static float fx = 525.0;
    static float fy = 525.0;
    static float cx = 319.5;
    static float cy = 239.5;

    Mat& camera_intrinsics = data.camera_intrinsics;
    camera_intrinsics = Mat::zeros(3, 3, CV_64F);
    camera_intrinsics.at<double>(0,0) = fx;
    camera_intrinsics.at<double>(0,2) = cx;
    camera_intrinsics.at<double>(1,1) = fy;
    camera_intrinsics.at<double>(1,2) = cy;
    camera_intrinsics.at<double>(2,2) = 1.f;
}

void FrameLoader::load(DataManager& data) {
    if (directory.find("rgbd_dataset_freiburg1_desk2_secret")!=string::npos)
    {
        loadCameraIntrinsics_TUM1(data);
        loadImgFileList(directory, begin_frame, end_frame, data);
    }else{
        loadCameraIntrinsics_kinect(data);
        loadImgFileList(directory+"/rgb/", begin_frame, end_frame, data);
    }
}
