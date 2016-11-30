//
//
// Monocular SLAM
// Following the pipeline of ORB_SLAM with optimisation framework implemented in OPT
//
// Xiaohan Jin, Yifan Xing, Yu Mao @ CMU
// 11/01/2016
//

#include <iostream>

#include "CommonCV.h"

#include "AppConfig.h"
#include "DataManager.h"
#include "Pipeline.h"
#include "FrameLoader.h"
#include "ORBFeatureExtractor.h"
#include "InitialCameraMotionEstimator.h"
#include "PointCloudVisualizer.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

	AppConfig config;
	config = parseArgs(argc, argv);

	// initialize a data manager
	DataManager dm;

	// Data loading
	FrameLoader frameLoader(config.inputDirectory, 0, 40);
	frameLoader.load(dm);

	// Pipeline Initialization
	ProcessingPipeline ORBSlam;
	ORBSlam.addStage(new ORBFeatureExtractor());
	ORBSlam.addStage(new InitialCameraMotionEstimator());
	// ORBSlam.addStage(new PointCloudVisualizer());
	
	// launch ORB-slam
	vector<Frame>& frames = dm.frames;
	for (int i=0; i<frames.size(); i++) {
		ORBSlam.process(dm, i);
	}

    return 0;
}
