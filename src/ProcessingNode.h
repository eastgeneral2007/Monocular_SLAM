//
// ProcessingNode.h
//
// ProcessingNode is is a single pipeline stage
// doing a specific task. It generates analysis 
// result and feed it into some data structs 
// in the data manager.
//
// @Yu

#ifndef PROCESSING_NODE
#define PROCESSING_NODE

#include "DataManager.h"

class ProcessingNode {
public:
	ProcessingNode(string name):name(name) {}
	virtual void init() {}
	virtual void finish() {}
	virtual void destroy() {}
	virtual ~ProcessingNode() {}
public:
	// will be called by batch processing pipeline
	virtual void process(DataManager& data) {}
	virtual bool validationCheck(DataManager& data) { return true; }
	// will be called by fused processing pipeline
	virtual void process(DataManager& data, int frameIdx) {}
	virtual bool validationCheck(DataManager& data, int frameIdx) { return true; }
public:
	string name;
};

#endif