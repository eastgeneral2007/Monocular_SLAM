//
// ProcessingEngine.h
//
// ProcessingEngine is is a single pipeline stage
// doing a specific task. It generates analysis 
// result and feed it into some data structs 
// in the data manager.
//
// @Yu

#ifndef PROCESSING_ENGINE
#define PROCESSING_ENGINE

#include "DataManager.h"

class ProcessingEngine {
public:
	ProcessingEngine(string name):name(name) {}
	virtual void init() {}
	virtual void finish() {}
	virtual void destroy() {}
	virtual ~ProcessingEngine() {}
public:
	// will be called by batch processing pipeline
	virtual void process(DataManager& data) {}
	virtual bool validationCheck(DataManager& data) { return true; }
	// will be called by fused processing pipeline
	virtual void process(DataManager& data, Frame& frame) {}
	virtual bool validationCheck(DataManager& data, Frame& frame) { return true; }
public:
	string name;
};

#endif