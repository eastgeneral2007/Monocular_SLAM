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
|-- ERL_frames.csv: csv file of <id, vectorised pose (3x4 row vectorised into 12x1)>
|-- out_info.txt: txt file of <frame id, frame_image_path>

*/

int main(int argc, char **argv) {
     
    DataManager dm;
    cv::Mat intrinsics_f1 = (Mat_<double>(3,3) << 517.306408,  0,   318.643040,
    0,   516.469215,   255.313989,
    0,   0,    1);
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

    vector<Frame> ERL_frames = Util::loadFramesOnlyIdRt(info_path+"/ERL_frames.csv");
    map<int, string> frame_id_to_imgfile = Util::loadFrameIdToImageFileName(info_path + "/out_info.txt");
    // assign intrinsics to each frame
    for (int i =0;i<ERL_frames.size();i++) {
        ERL_frames[i].K = intrinsics_f1;
#ifdef DEBUG        
        cout << "ERL_frames[" << i << "]'s Rt:\n" << ERL_frames[i].Rt << endl;
#endif 
    }

    dm.frames = ERL_frames;

    // Pipeline for ERl visualisation
	ProcessingPipeline ERLVisualisation;
	// ERLVisualisation.addStage(new TrajectoryVisualizer());
	ERLVisualisation.addStage(new PointCloudVisualizer());
	
	// launch the pipeline
	for (int i=0; i<dm.frames.size(); i++) {
		// process
        ERLVisualisation.process(dm, i);
        
        // visualise the actual frame image at this timestamp
        cv::Mat this_stamp_im = imread(image_path + "/" + frame_id_to_imgfile[dm.frames[i].meta.frameID], CV_LOAD_IMAGE_COLOR);
        imshow("this_stamp_im", this_stamp_im);
        waitKey(10);
	}
}
