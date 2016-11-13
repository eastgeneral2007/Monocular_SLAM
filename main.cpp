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

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

	AppConfig config;
	config = parseArgs(argc, argv);

	// initialize a data manager
	DataManager dm;

	// build pipeline
	BatchProcessingPipeline dataLoader;
	dataLoader.addStage(new FrameLoader(config.inputDirectory, 0, 20));
	
	FusedProcessingPipeline ORBSlam;
	ORBSlam.addStage(new ORBFeatureExtractor());
	
	// loading data
	dataLoader.process(dm);
	
	// launch ORB-slam
	vector<Frame>& frames = dm.frames;
	for (int i=0; i<frames.size(); i++) {
		ORBSlam.process(dm, frames[i]);
	}

    return 0;
}
