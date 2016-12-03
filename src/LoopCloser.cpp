//
// Created by JimXing on 13/11/16.
//

#include "LoopCloser.h"


LoopCloser::LoopCloser(const string &name, DataManager &data_reference) : ProcessingNode(name), data_reference(data_reference) { }

void LoopCloser::Run()
{
    if (DetectLoop()) {
        if (ComputeSim3()) {
            CorrectLoop();
        }
    }
}

bool LoopCloser::DetectLoop()
{
    vector<Frame> frames = data_reference.frames;
    vector<vector<float> > distances;
    vector<vector<int> > indices;
    int n = 10;
    int count_max = 0;
    float dist_thr = 0.05;
    int ind_max = 0;
    int i = frames.size();
    Mat descriptors1 = frames[i].features.descriptors;
    for (int j = 1; j< frames.size()-1;j++)
    {
        Mat descriptors2 = frames[j].features.descriptors;
        NBestMatches(descriptors1, descriptors2, n, distances, indices);
        int count = 0;
        for(unsigned int di=0; di<distances.size(); ++di)
        {
            for(unsigned int dj=0; dj<distances.at(i).size(); ++dj)
            {
                if(distances.at(di).at(dj) < dist_thr)
                    count ++;
            }
        }
        if (count > count_max)
        {
            ind_max = j;
            count_max = count;    
        }
    }
    printf("Loop candidates: &d, &d", &i, &ind_max);
    return true;
}

void LoopCloser::NBestMatches(Mat descriptors1, Mat descriptors2, unsigned int n, vector<vector<float> > & distances, vector<vector<int> > & indices)
{
    // get enough space to create n best matches
    distances.clear();
    distances.resize(descriptors1.rows);
    indices.clear();
    indices.resize(descriptors1.rows);

    for(int i=0; i<descriptors1.rows; ++i)
    {
        // references to current elements:
        std::vector<float> & cDistances = distances.at(i);
        std::vector<int>  & cIndices = indices.at(i);
        cDistances.resize(n,FLT_MAX);
        cIndices.resize(n,-1); 

        // now find the 3 best matches for descriptor i:
        for(int j=0; j<descriptors2.rows; ++j)
        {
            float euclideanDistance = 0;
            for( int dim = 0; dim < descriptors1.cols; ++dim)
            {
                float tmp = descriptors1.at<float>(i,dim) - descriptors2.at<float>(j, dim);
                euclideanDistance += tmp*tmp;
            }
            euclideanDistance = sqrt(euclideanDistance);

            float tmpCurrentDist = euclideanDistance;
            int tmpCurrentIndex = j;

            // update current best n matches:
            for(unsigned int k=0; k<n; ++k)
            {
                if(tmpCurrentDist < cDistances.at(k))
                {
                    int tmpI2 = cIndices.at(k);
                    float tmpD2 = cDistances.at(k);

                    // update current k-th best match
                    cDistances.at(k) = tmpCurrentDist;
                    cIndices.at(k) = tmpCurrentIndex;

                    // previous k-th best should be better than k+1-th best
                    tmpCurrentDist = tmpD2;
                    tmpCurrentIndex =tmpI2;
                }
            }


        }
    }

}


void LoopCloser::TestUnit()
{
    cv::Mat input = cv::imread("../data/rgbd_dataset_freiburg1_desk2_secret/1305033146.744789.png");

    cv::Mat gray;
    cv::cvtColor(input, gray, CV_BGR2GRAY);

    cv::SiftFeatureDetector detector( 7500 );
    cv::SiftDescriptorExtractor describer;

    std::vector<cv::KeyPoint> keypoints;

    detector.detect( gray, keypoints );
    cv::drawKeypoints(input,keypoints,input);

    cv::Mat descriptors;
    describer.compute(gray, keypoints, descriptors);

    int n = 4;
    std::vector<std::vector<float> > dists;
    std::vector<std::vector<int> > indices;

    // compute the N best matches between the descriptors.
    NBestMatches(descriptors, descriptors, n, dists, indices);

    for(unsigned int i=0; i<dists.size(); ++i)
    {
        for(unsigned int j=0; j<dists.at(i).size(); ++j)
        {
            if(dists.at(i).at(j) < 0.05)
                cv::line(input, keypoints[i].pt, keypoints[indices.at(i).at(j)].pt, cv::Scalar(255,255,255) );
        }
    }

    cv::imshow("input", input);
    cv::waitKey(0);
}


bool LoopCloser::ComputeSim3()
{
    return false;
}

void LoopCloser::CorrectLoop()
{
    return;
}