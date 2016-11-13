//
// FrameLoader.h
//
// Frame loader loaded all the frames from the directory
// and feed them into frames.
//
// @Yu

#include "ProcessingEngine.h"

class FrameLoader: public ProcessingEngine
{
public:
	FrameLoader(string directory, 
				int begin_frame, 
			    int end_frame):
	ProcessingEngine("FrameLoader"), 
	directory(directory),
	begin_frame(begin_frame),
	end_frame(end_frame) {}
	
	virtual void process(DataManager& data);
private:
	string directory;
	int begin_frame;
	int end_frame;
};