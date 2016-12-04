//
// FrameLoader.h
//
// Frame loader loaded all the frames from the directory
// and feed them into frames.
//
// @Yu

class FrameLoader
{
public:
	FrameLoader(string directory, 
				int begin_frame, 
			    int end_frame,
				int step):
	directory(directory),
	begin_frame(begin_frame),
	end_frame(end_frame),
	step(step) {}
	
	void load(DataManager& data);
private:
	string directory;
	int begin_frame;
	int end_frame;
	int step;
};