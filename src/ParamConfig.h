#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

// feature matching
const static float FEATURE_MATCH_RATIO_TEST = 0.85f;

// adjustable parameters for huber cost function
const float THRESH_HUBER_FULL_BA = sqrt(5.99);

const float THRESH_HUBER = sqrt(5.991); 

const float CHI2_THRESH = 5.991;

// default number of iterations for pose BA
const int POSE_BA_ITER = 10;

// tuned number of iterations for full BA
const int FULL_BA_ITER = 15;

// parameter for SURF detector 
const int SURF_MIN_HESSIAN = 400;

// Fundamental matrix parameters
const double MAX_DISTANCE = 3.;
const double CONFIDENCE = 0.85;

#endif