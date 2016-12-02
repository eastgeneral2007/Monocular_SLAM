#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

// feature matching
const static float FEATURE_MATCH_RATIO_TEST = 0.8f;

// adjustable parameters for huber cost function
const float THRESH_HUBER = sqrt(5.991); 

const float CHI2_THRESH = 5.991;

// default number of iterations for pose BA
const int POSE_BA_ITER = 10;

#endif