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

int main(int argc, char **argv) {
     
    DataManager dm;
    cv::Mat intrinsics_f1 = (Mat_<double>(3,3) << 525.0,  0,   319.5000,
    0,   525.0,   239.5000,
    0,   0,    1);
    // load the .csv frames
    if (argc < 2) {
        printf("\n");
        printf("Usage:  <input_frame_directory>\n");
        printf("\n");
        exit(-1);
    }
    cout << "Input path:" << argv[1] << endl;
    vector<Frame> ERL_frames = Util::loadFramesOnlyIdRt(argv[1]);
    // assign intrinsics to each frame
    for (int i =0;i<ERL_frames.size();i++) {
        ERL_frames[i].K = intrinsics_f1;
        cout << "ERL_frames[" << i << "]'s Rt:\n" << ERL_frames[i].Rt << endl; 
    }

    dm.frames = ERL_frames;

    // Pipeline for ERl visualisation
	ProcessingPipeline ERLVisualisation;
	// ERLVisualisation.addStage(new TrajectoryVisualizer());
    Mat for_stop = Mat::zeros(384, 512, CV_64F);
	ERLVisualisation.addStage(new PointCloudVisualizer());
	
	// launch the pipeline
	for (int i=0; i<dm.frames.size(); i++) {
		ERLVisualisation.process(dm, i);
        imshow("for_stop", for_stop);
        waitKey(10);
	}
}
