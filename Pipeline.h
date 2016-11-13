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

class ProcessingPipeline
{
public:
	void process(DataManager &data) {

		// initialize the engine
		for (ProcessingEngine* engine : stages) {
			engine->init();
		}

		// start the pipeline
		for (ProcessingEngine* engine : stages) {
			if (!engine->validationCheck()) {
				std::cout << "Engine [" << engine->name << "]: validation check failed."<< std::endl;
				return;
			}
			engine->process(data);
		}
	}

	void addStage(ProcessingEngine *engine) {
		stages.push_back(engine);
	}

	~ProcessingPipeline() {
		for (ProcessingEngine* engine : stages) {
			engine->destroy();
			delete engine;
		}
	}
	
private:
	vector <ProcessingEngine *> stages;
};

#endif