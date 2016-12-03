//
// PointCloudVisualizer.h
//
// PointCloudVisualizer is a visualization tool 
// for rendering the result of 3D reconstruction.
//
// @Yu

#ifndef POINTCLOUDVISUALIZER_H
#define POINTCLOUDVISUALIZER_H

#include "Common.h"
//#include "CommonCV.h"
#include "CommonBoost.h"
#include "ProcessingNode.h"

namespace pcl {namespace visualization {class PCLVisualizer;}}
class PointCloudVisualizer : public ProcessingNode
{
public:
    PointCloudVisualizer():ProcessingNode("PointCloudVisualizer"){}
    virtual void init();
	virtual void process(DataManager& data, int frameIdx);
	virtual bool validationCheck(DataManager& data, int frameIdx);
private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif