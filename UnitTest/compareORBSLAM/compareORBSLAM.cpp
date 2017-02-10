#include <iostream>
#include "../../src/CommonCV.h"
#include "../../src/DataManager.h"
#include "../../src/DataManager.h"
#include "../../src/Pipeline.h"
#include "../../src/PointCloudVisualizer.h"
#include "../../src/TrajectoryVisualizer.h"
#include "../../src/Util.h"
using namespace std;
using namespace cv;

#define DEBUG_LOG

/**
Input required format:

<input_frame_directory>
|-- rgb/: folder containing actual images
|-- rgb.txt: txt file containing <frame number, image_path>

<input_info_directory>
|-- out_info.txt: txt file of <frame id, frame_image_path>
|-- frames_ORB.csv: csv file of <frame id, vectorised pose (3x4 row vectorised into 12x1)>
|-- ORB_{i}_frames.csv: csv files of all keyframes at timstamp of frame i
        Format: Repeat: row of <frame id, vectorised pose (3x4 row vectorised into 12x1)>
                        rows of <2d_feature_u, 2d_feature_v, 2d_feature_scale>
                        <space>
                        rows of associated mappoint id (-1 if not associated with any)
                        <space>
|-- ORB_{i}_mappoint.csv: csv files of mappoints at timestamp of frame i
        Format: Repeat: row of <mapppoint id, X, Y, Z>
                        rows of <observing frame id, corresponding feature idx in that frame>
                        <space>
*/


int main(int argc, char **argv) {
    DataManager dm;
    cv::Mat intrinsics_TUM1 = (Mat_<double>(3,3) << 517.306408,  0,   318.643040,
    0,   516.469215,   255.313989,
    0,   0,    1);
    // load the .csv frames
    if (argc < 3) {
        printf("\n");
        printf("Usage:  <input_frame_directory> <input_info_directory>\n");
        printf("\n");
        exit(-1);
    }
#ifdef DEBUG_LOG
    cout << "Image Input path:" << argv[1] << endl;
    cout << "Info Input path:" << argv[2] << endl;
#endif
    string image_path = argv[1];
    string info_path = argv[2];
    string info_file = "out_info.txt";
    string map_suffix = "_mappoint.csv";
    string frame_suffix = "_frames.csv";
    string ORB_prefix = "ORB_"; 


//     // load results from ORB SLAM
//     vector<Frame> ORB_frames = Util::loadFramesOnlyIdRt(info_path + "/frames_ORB.csv");

    // load frame id to file name 
    map<int, string> frame_id_to_imgfile = Util::loadFrameIdToImageFileName(info_path + "/" +info_file);

    // Pipeline
    ProcessingPipeline ORBSLAMComparator;
    // ORBSLAMComparator.addStage(new TrajectoryVisualizer());
    ORBSLAMComparator.addStage(new PointCloudVisualizer());

    // launch the pipeline
    for (int i=0; i<frame_id_to_imgfile.size(); i++) {
        
        // load the cur mappoints at this time stamp
        vector<MapPoint> this_stamp_mappoints = Util::loadMapPoints(info_path + "/"+ ORB_prefix + to_string(i) + map_suffix);
        dm.mapPoints = this_stamp_mappoints;
#ifdef DEBUG_LOG        
        cout << "this timestamp mapPoints size:" << dm.mapPoints.size() << endl;
#endif  

        // load the keyframes at this timestamp
        vector<Frame> this_stamp_frames = Util::loadFrames(info_path + "/" + ORB_prefix + to_string(i) + frame_suffix);
        dm.frames = this_stamp_frames;
#ifdef DEBUG_LOG        
        cout << "this timestamp frames size:" << dm.frames.size() << endl;
#endif
        // load the frame meta 
        cv::Mat this_im = imread(image_path + "/" + frame_id_to_imgfile[i], CV_LOAD_IMAGE_COLOR);
#ifdef DEBUG_LOG     
        cout << "image path:" << image_path + "/" + frame_id_to_imgfile[i] << endl;
#endif  
        // assign intrinsics to each frame
        for (int j =0;j<dm.frames.size();j++) {
            dm.frames[j].K = intrinsics_TUM1;
#ifdef DEBUG_LOG
            cout << "ORB_frames[" << j << "]'s Rt:\n" << dm.frames[j].Rt << endl;
#endif         

        dm.frames[j].frameBuffer = this_im;
        }
        
        //note: dm.frames.size() will be < i as we only record keyFrames here
        
        // main process
	ORBSLAMComparator.process(dm, i);
        imshow("im for current time stamp", this_im);
        waitKey(0);
	}

}
