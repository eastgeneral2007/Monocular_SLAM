#include "FrameLoader.h"

bool has_suffix(const string& s, const string& suffix)
{
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

Frame parseImgRawFileName(string directory, string filename)
{
    double timestamp;
    sscanf(filename.c_str(), "%lf.png", &timestamp);
    Frame f; 
    FrameMeta& meta = f.meta;
    meta.timestamp = timestamp;
    meta.framename = directory + filename;
    return f;
}

void loadImgFileList(string directory, int begin_frame, int end_frame, DataManager& data) {
    if (directory.back()!='/') {
        directory += '/';
    }

    DIR *pDIR;
    int index=0;
    struct dirent *entry;
    vector<Frame> imgNamelist;
    if( (pDIR=opendir(directory.c_str())) !=NULL){
        while((entry = readdir(pDIR))!= NULL){
            if( has_suffix(entry->d_name, ".png") || has_suffix(entry->d_name, ".jpg") ) {
                imgNamelist.push_back(parseImgRawFileName(directory, entry->d_name));
                index++;
            }
        }
        closedir(pDIR);
    }
    sort(imgNamelist.begin(), imgNamelist.end());

    for (int i=begin_frame; i<MIN(end_frame, imgNamelist.size()); i++){
        Frame f;
        FrameMeta& meta = f.meta;
        meta.timestamp=imgNamelist[i].meta.timestamp;
        meta.framename=imgNamelist[i].meta.framename;
        meta.frameID=i;
        f.frame=imread(imgNamelist[i].meta.framename, CV_LOAD_IMAGE_UNCHANGED);
        imshow("frame", f.frame);
        waitKey();
        data.frames.push_back(f);
    }
}

void FrameLoader::process(DataManager& data) {
	loadImgFileList(directory, begin_frame, end_frame, data);
}
