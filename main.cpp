//
//
// Monocular SLAM
// Following the pipeline of ORB_SLAM with optimisation framework implemented in OPT
//
// Xiaohan Jin, Yifan Xing, Yu Mao @ CMU
// 11/01/2016
//

#include <iostream>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "AppConfig.h"
#include "DataManager.h"
#include "Pipeline.h"
#include "FrameLoader.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

	AppConfig config;
	config = parseArgs(argc, argv);

	// initialize a data manager
	DataManager dm;

	// build the ORB SLAM pipeline
	ProcessingPipeline ORBSlam;
	ORBSlam.addStage(new FrameLoader(config.inputDirectory, 0, 20));
	
	// start processing
	ORBSlam.process(dm);

    return 0;
}
