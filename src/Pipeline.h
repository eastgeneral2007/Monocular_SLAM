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
#include "ProcessingEngine.h"

//
// BaseProcessingPipeline
//
// The base processing pipeline manages
// a sequence of processing stages(engine).
//
class BaseProcessingPipeline
{
public:
	vector <ProcessingEngine *> stages;

public:
	void addStage(ProcessingEngine *engine) {
		stages.push_back(engine);
	}

	virtual ~BaseProcessingPipeline() {
		for (ProcessingEngine* engine : stages) {
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
class FusedProcessingPipeline: public BaseProcessingPipeline
{
public:
	void process(DataManager &data, Frame& frame) {

		// initialize the engine
		for (ProcessingEngine* engine : stages) {
			engine->init();
		}

		// start the pipeline
		for (ProcessingEngine* engine : stages) {
			if (!engine->validationCheck(data, frame)) {
				std::cout << "Engine [" << engine->name << "]: validation check failed."<< std::endl;
				return;
			}
			engine->process(data, frame);
		}
	}

};

//
// BatchProcessingPipeline
//
// BatchProcessingPipeline is used for pre-processing
// and post-processing steps such as data loading and
// initialization.
//
class BatchProcessingPipeline: public BaseProcessingPipeline
{
public:
	void process(DataManager &data) {

		// initialize the engine
		for (ProcessingEngine* engine : stages) {
			engine->init();
		}

		// start the pipeline
		for (ProcessingEngine* engine : stages) {
			if (!engine->validationCheck(data)) {
				std::cout << "Engine [" << engine->name << "]: validation check failed."<< std::endl;
				return;
			}
			engine->process(data);
		}
	}
};

#endif