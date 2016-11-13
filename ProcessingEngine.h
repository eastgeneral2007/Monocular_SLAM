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
	virtual bool validationCheck() { return true; }
	virtual void process(DataManager& data) = 0;
	virtual void finish() {}
	virtual void destroy() {}
	virtual ~ProcessingEngine() {}
public:
	string name;
};

#endif