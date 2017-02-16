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

// #define DEBUG_LOG

/**
Input required format:

<input_frame_directory>
|-- folder containing actual images

<input_info_directory>
|-- frame_pose.csv: csv file of <id, vectorised pose (3x4 row vectorised into 12x1)>
|-- out_info.txt: txt file of <frame id, frame_image_path(relative to input_frame_directory)>

*/

int main(int argc, char **argv) {
     
    DataManager dm;
    cv::Mat intrinsics_f1 = (Mat_<double>(3,3) << 315.5,  0,   376,
    0,   315.5,   240,
    0,   0,    1); // svo sin data set

    // load the .csv frames
    if (argc < 3) {
        printf("\n");
        printf("Usage:  <input_frame_directory> <input_info_directory>\n");
        printf("\n");
        exit(-1);
    }
#ifdef DEBUG
    cout << "Input path:" << argv[1] << endl;
    cout << "Info path:" << argv[2] << endl;
#endif
    string image_path = argv[1];
    string info_path = argv[2];

    vector<Frame> externel_frames = Util::loadFramesOnlyIdRt(info_path+"/frame_pose.csv");
    map<int, string> frame_id_to_imgfile = Util::loadFrameIdToImageFileName(info_path + "/out_info.txt");
    // assign intrinsics to each frame
    for (int i =0;i<externel_frames.size();i++) {
        externel_frames[i].K = intrinsics_f1;
#ifdef DEBUG        
        cout << "frames[" << i << "]'s Rt:\n" << externel_frames[i].Rt << endl;
#endif 
    }

    dm.frames = externel_frames;

    // Pipeline for visualisation
	ProcessingPipeline ExternelVisualisation;
	// ExternelVisualisation.addStage(new TrajectoryVisualizer());
	ExternelVisualisation.addStage(new PointCloudVisualizer());
	
	// launch the pipeline
	for (int i=0; i<dm.frames.size(); i++) {
		// process
        ExternelVisualisation.process(dm, i);
        
        // visualise the actual frame image at this timestamp
        cv::Mat this_stamp_im = imread(image_path + "/" + frame_id_to_imgfile[dm.frames[i].meta.frameID], CV_LOAD_IMAGE_COLOR);
        imshow("this_stamp_im", this_stamp_im);
        waitKey(10);
	}
}
