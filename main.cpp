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

using namespace std;
using namespace cv;

static string input_stream_directory;

int main(int argc, char **argv) {

	AppConfig config;
	config = parseArgs(argc, argv);

    DataManager dm;
    dm.loadImgFileList(config.inputDirectory, 0, 20);

    return 0;
}
