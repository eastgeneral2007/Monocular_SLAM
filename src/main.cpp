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
#include "FeatureExtractor.h"
#include "CameraPoseEstimator.h"
#include "PointCloudVisualizer.h"
#include "Optimiser.h"
#include "TrajectoryVisualizer.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	AppConfig config;
	config = parseArgs(argc, argv);

	// initialize a data manager
	DataManager dm;

	// Data loading
	int begin_frame = 0, end_frame = 200, step = 2;
	FrameLoader frameLoader(config.inputDirectory, begin_frame, end_frame, step);
	frameLoader.load(dm);

	// Pipeline Initialization
	ProcessingPipeline ORBSlam;
	ORBSlam.addStage(new FeatureExtractor());
	ORBSlam.addStage(new CameraPoseEstimator());
	// ORBSlam.addStage(new Optimiser()); // this is for full BA, to turn on pose BA, uncomment off invokation in pnpPoseEstimation()
	ORBSlam.addStage(new TrajectoryVisualizer());
	ORBSlam.addStage(new PointCloudVisualizer());
	
	// launch ORB-slam
	vector<Frame>& frames = dm.frames;
	for (int i=0; i<frames.size(); i++) {
		ORBSlam.process(dm, i);
	}

    return 0;
}
