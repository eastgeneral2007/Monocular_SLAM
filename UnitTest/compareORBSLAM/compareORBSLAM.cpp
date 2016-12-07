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

    // load results from ORB SLAM
    vector<Frame> ORB_frames = Util::loadFramesOnlyIdRt(info_path + "/frames_ORB.csv");
    // assign intrinsics to each frame
    for (int i =0;i<ORB_frames.size();i++) {
        ORB_frames[i].K = intrinsics_TUM1;
#ifdef DEBUG_LOG
        cout << "ORB_frames[" << i << "]'s Rt:\n" << ORB_frames[i].Rt << endl;
#endif         
    }

    // load frame id to file name 
    map<int, string> frame_id_to_imgfile = Util::loadFrameIdToImageFileName(info_path + "/out_info.txt");

    dm.frames = ORB_frames;

    // Pipeline
	ProcessingPipeline ORBSLAMComparator;
	// ORBSLAMComparator.addStage(new TrajectoryVisualizer());
	ORBSLAMComparator.addStage(new PointCloudVisualizer());
	
	// launch the pipeline
	for (int i=0; i<dm.frames.size(); i++) {
        // load the cur mappoints at this time stamp
        vector<MapPoint> this_stamp_mappoints = Util::loadMapPointsOnlyIdXYZ(info_path + "/map_point_ORB_"+to_string(i)+".csv");
        dm.mapPoints = this_stamp_mappoints;
#ifdef DEBUG_LOG        
        cout << "this timestamp mapPoints size:" << dm.mapPoints.size() << endl;
#endif  
        // load the frame meta 
        cv::Mat this_im = imread(image_path + "/" + frame_id_to_imgfile[dm.frames[i].meta.frameID], CV_LOAD_IMAGE_COLOR);
#ifdef DEBUG_LOG     
        cout << "image path:" << image_path + "/" + frame_id_to_imgfile[dm.frames[i].meta.frameID] << endl;
#endif  
        dm.frames[i].frameBuffer = this_im;
        
        // process
	ORBSLAMComparator.process(dm, i);
        imshow("im for current time stamp", this_im);
        waitKey(10);
	}
}
