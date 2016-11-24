//
// FrameLoader.h
//
// Frame loader loaded all the frames from the directory
// and feed them into frames.
//
// @Yu

#include "ProcessingNode.h"

class FrameLoader: public ProcessingNode
{
public:
	FrameLoader(string directory, 
				int begin_frame, 
			    int end_frame):
	ProcessingNode("FrameLoader"), 
	directory(directory),
	begin_frame(begin_frame),
	end_frame(end_frame) {}
	
	virtual void process(DataManager& data) override;
private:
	string directory;
	int begin_frame;
	int end_frame;
};