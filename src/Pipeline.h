//
// ProcessingPipeline.h
//
// ProcessingPipeline is a pipeline of processing engine.
// We will be constructing a SLAM pipeline by defining
// each stages of processing.
//
// @Yu

#ifndef PROCESSING_PIPELINE_H
#define PROCESSING_PIPELINE_H

#include "DataManager.h"
#include "ProcessingNode.h"

//
// BaseProcessingPipeline
//
// The base processing pipeline manages
// a sequence of processing stages(engine).
//
class BaseProcessingPipeline
{
public:
	vector <ProcessingNode *> stages;

public:
	void addStage(ProcessingNode *engine) {
		stages.push_back(engine);
	}

	virtual ~BaseProcessingPipeline() {
		for (ProcessingNode* engine : stages) {
			engine->destroy();
			delete engine;
		}
	}
};

// 
// FusedProcessingPipeline
//
// FusedProcessingPipeline process the data
// in a frame by frame manner.
// 
class ProcessingPipeline: public BaseProcessingPipeline
{
public:
	void process(DataManager &data, int frameIdx) {
		
		if(frameIdx == 0) {
			for (ProcessingNode* node : stages) {
				node->init();
			}
		}

		for (ProcessingNode* node : stages) {
			if (!node->validationCheck(data, frameIdx)) {
				cout << "Node [" << node->name << "]: validation check failed."<< endl;
				return;
			}
			// cout << node->name << " is processing frame #" << frameIdx << endl;
			node->process(data, frameIdx);
		}
	}

};

#endif