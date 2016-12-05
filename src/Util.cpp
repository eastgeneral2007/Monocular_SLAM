//
// Created by JimXing on 12/11/16.
//

#include "Util.h"
#include "Commong2o.h"
#include <map>
#include "Converter.h"
#include "ParamConfig.h"
#include <set>

// #define DEBUG_BA
#define DEBUG_POSEBA 

using namespace std;
using namespace g2o;

typedef BlockSolver< BlockSolverTraits<6, 3> > SLAMBlockSolver;
typedef LinearSolverEigen<BlockSolver_6_3::PoseMatrixType> SLAMLinearSolver;
// typedef BlockSolver< BlockSolverTraits<6, 3> > ::LinearSolverType SLAMLinearSolver;

typedef std::map<int, int>::iterator map_frame_iter;

cv::Mat Util:: ComputeF(Frame f1, Frame f2) {
    return cv::Mat();
}

void Util::GlobalBundleAdjustemnt(DataManager &data, int n_iterations, bool pb_stop_flag,
                                  const unsigned long n_loop_kf, const bool b_robust) {
    // just invoke the usual bundle adjustment function with all map points and key frames (that has Rt) maintained in data manager
    Util::BundleAdjustment(data, data.frames, data.mapPoints, n_iterations,pb_stop_flag,n_loop_kf,b_robust);
}

void Util::BundleAdjustment(DataManager &data, vector<Frame> &frames, vector<MapPoint> &map_points,
                             int n_iterations, bool pb_stop_flag, const unsigned long n_loop_kf,
                             const bool b_robust) {

#ifdef DEBUG_BA
    cout << "Starting full BA, with configuration: n_iterations: " << n_iterations 
    << "pb_stop_flag:" << pb_stop_flag << " n_loop_kf: " << n_loop_kf << " b_robust" << b_robust << endl; 
#endif

    SparseOptimizer optimizer;

    SLAMLinearSolver * linear_solver;
    linear_solver = new LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>();
    // linear_solver->setBlockOrdering(false);

    SLAMBlockSolver  *block_solver = new SLAMBlockSolver(linear_solver);

    OptimizationAlgorithmLevenberg *algo = new OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(algo);

    int frame_max_id = 0; // hold the most recent frame id
    set<long unsigned int> frame_vertex_id_set;

#ifdef DEBUG_BA
    cout << "frames.size():" << frames.size()<< endl;
#endif

    // create vertex for Frames
    for (int i =0;i<frames.size();i++) {
        Frame * this_kf = &frames[i];
        // only if the frame has already been processed (i.e., has Rt)
        if (!this_kf->Rt.empty()) {
#ifdef DEBUG_BA
            cout << "create vertex for frame:" << this_kf->meta.frameID << endl;
#endif
            g2o::VertexSE3Expmap * frame_vertex = new g2o::VertexSE3Expmap();
            frame_vertex->setEstimate(Converter::cvMatToSE3Quat(this_kf->Rt));
            frame_vertex->setId(this_kf->meta.frameID);
            frame_vertex->setFixed(this_kf->meta.frameID ==0); // fix the first frame
            optimizer.addVertex(frame_vertex);
            if(this_kf->meta.frameID > frame_max_id)
                frame_max_id=this_kf->meta.frameID;
            frame_vertex_id_set.insert(this_kf->meta.frameID);
        }
    }

#ifdef DEBUG_BA
    cout << "finish creating vertex for frames" << endl;
    cout << "frame_max_id:" << frame_max_id << endl;
#endif

    int optimized[map_points.size()];
    // create vertex for MapPoints
    for (int i =0;i<map_points.size();i++) {
        MapPoint *this_map_point = &map_points[i];
        map<int, int> &observations = this_map_point->observerToIndex; // get all observers mapping

        map_frame_iter iterator;
        // check if it does not have associated frame in the frame_vertex_id_set
        for (iterator = observations.begin(); iterator != observations.end(); iterator++) {
            const Frame *observer = Util::findFrameById(data.frames, iterator->first);
            if (observer != NULL) {
                if (frame_vertex_id_set.find(observer->meta.frameID) != frame_vertex_id_set.end()) {
                    break;
                }
            }
        }
        // ignore this mapPoint, if no associsations with frame inserted to optimizer
        if (iterator == observations.end()) {
#ifdef DEBUG_BA
            cout << "map point id:"<< this_map_point->id << "has no observer in the given frames"<< endl;
#endif
            optimized[i] = false;
            continue;
        }

#ifdef DEBUG_BA
        cout << "create vertext for map point id:"<< this_map_point->id << endl;
#endif
        optimized[i] = true;

        g2o::VertexSBAPointXYZ *point_vertex = new g2o::VertexSBAPointXYZ();
        Eigen::Matrix<double,3,1> xyz = Converter::point3dToVector3d(this_map_point->worldPosition);
#ifdef DEBUG_BA
        cout << "vector point 3d for this map point " << xyz << endl; 
#endif
        point_vertex->setEstimate(xyz);
        
        // id for point vertex, starting from frame_max_id + 1 to not overlap with frame_vertex ids
        int map_point_id = this_map_point->id + frame_max_id + 1;
        point_vertex->setId(map_point_id);
        point_vertex->setMarginalized(true);
        optimizer.addVertex(point_vertex);

        // for each added MapPoint, create edge from its observers
        for (map_frame_iter iterator = observations.begin(); iterator != observations.end(); iterator++) {
            
            const Frame *observer = Util::findFrameById(data.frames, iterator->first);
            int feature_idx = iterator->second;
            // ignore if the observer not found or is not in frame vertexes added
            if (observer != NULL && frame_vertex_id_set.find(observer->meta.frameID) != frame_vertex_id_set.end()) {

                const cv::Point2f &pos = observer->features.positions[feature_idx];

                Eigen::Matrix<double, 2, 1> obs;
                obs << pos.x, pos.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(map_point_id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(observer->meta.frameID)));
#ifdef DEBUG_BA                
                cout << observer->Rt << endl;
                g2o::VertexSE3Expmap* temp_vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(observer->meta.frameID));
                cout << temp_vertex->estimate() << endl;
#endif                
                e->setMeasurement(obs);
                float this_keypoint_scale = observer->features.scales[feature_idx]; // TODO: change to the actual scale that this keypoint is detected
                // cout << Eigen::Matrix2d::Identity() * (1.0f/this_keypoint_scale) << endl;
                e->setInformation(Eigen::Matrix2d::Identity() * (1.0f/this_keypoint_scale));

                if (b_robust) {
                    g2o::RobustKernelHuber *huber_kernel = new g2o::RobustKernelHuber;
                    e->setRobustKernel(huber_kernel);
                    huber_kernel->setDelta(THRESH_HUBER_FULL_BA);
                }

                e->fx = observer->K.at<double>(0,0);
                e->fy = observer->K.at<double>(1,1);
                e->cx = observer->K.at<double>(0,2);
                e->cy = observer->K.at<double>(1,2);

                optimizer.addEdge(e);
            }
        }
    }
    // end of construction of graph for BA


#ifdef DEBUG_BA
    cout << "end of construction of graph for BA, start optimisation" << endl;
#endif

    // start optimisation
    optimizer.initializeOptimization();
    optimizer.optimize(n_iterations);

    // Recover optimized data

    //Keyframes
    for(int i=0; i<frames.size(); i++) {
        
        Frame * this_frame = &frames[i];
        // only recover those that has Rt
        if (!this_frame->Rt.empty()) {
            g2o::VertexSE3Expmap* frame_vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(this_frame->meta.frameID));
            g2o::SE3Quat SE3quat = frame_vertex->estimate();
            if(n_loop_kf==0) {
#ifdef DEBUG_BA                
                cout << "original Rt for frame" << this_frame->meta.frameID << " = \n" << this_frame->Rt << endl;
                cout << "estimated Rt for frame" << this_frame->meta.frameID << " = \n" << Converter::SE3QuatToCvMat(SE3quat) << endl;
#endif          
                this_frame->Rt = Converter::SE3QuatToCvMat(SE3quat);      
            }
            // TODO: check if needs to maintain different poses for n_loop_kf != 0, loop points   
        }
    }

    //Map Points
    for(int i=0; i<map_points.size(); i++) {
        if(optimized[i]) {
            MapPoint* this_map_point = &map_points[i];

            g2o::VertexSBAPointXYZ* point_vertex = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(this_map_point->id+frame_max_id+1));

            if(n_loop_kf==0) {
#ifdef DEBUG_BA                 
                cout << "original worldPosition for map_point: "<< this_map_point->id  << "\n" << this_map_point ->worldPosition << endl;
                cout << "estimated worldPosition:"<< this_map_point->id << "\n" << Converter::vector3dToPoint3d(point_vertex->estimate()) << endl;
#endif          
                this_map_point->worldPosition = Converter::vector3dToPoint3d(point_vertex->estimate());
      
            }
            // TODO: check if needs to maintain different 3D coords for n_loop_kf != 0, loop points
        }
    }
}

void Util::PoseBundleAdjustment(Frame &frame, DataManager &data, int n_round) {
#ifdef DEBUG_POSEBA
    cout << "Starting pose BA, with configuration: n_iterations: " << n_round 
    << " at frame:" << frame.meta.frameID << endl;
#endif
    
    SparseOptimizer optimizer;
    
    BlockSolver_6_3::LinearSolverType * linear_solver;
    linear_solver = new LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>();
    // linear_solver->setBlockOrdering(false);
    
    SLAMBlockSolver  *block_solver = new SLAMBlockSolver(linear_solver);

    OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(algo);

    // insert the frame vertex
    g2o::VertexSE3Expmap * frame_vertex = new g2o::VertexSE3Expmap();
    // cout << "frame.Rt:\n" << frame.Rt << endl;
    // cout << "frame.Rt quoternian:\n" << Converter::cvMatToSE3Quat(frame.Rt) << endl;

    frame_vertex->setEstimate(Converter::cvMatToSE3Quat(frame.Rt));
    frame_vertex->setId(0); // since only one frame here
    frame_vertex->setFixed(false);
    optimizer.addVertex(frame_vertex);

    int num_map_points = frame.features.mapPointsIndices.size();
    int num_correspondences = 0;
    // keep a reference to the added edges
    vector<g2o::EdgeSE3ProjectXYZOnlyPose * > added_edges;
    map<g2o::EdgeSE3ProjectXYZOnlyPose *, bool> is_outlier;
    added_edges.reserve(num_map_points);

    // insert MapPoint as EdgeSE3ProjectXYZOnlyPose
    for (int i =0;i < num_map_points;i++) {
        int this_associated_map_point_idx = frame.features.mapPointsIndices[i];
        if (this_associated_map_point_idx != -1) {
            MapPoint * this_map_point = findMapPointById(data.mapPoints, this_associated_map_point_idx);
            if (this_map_point != NULL) {
                g2o::EdgeSE3ProjectXYZOnlyPose * e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                num_correspondences ++;
                
                int corresponding_feature_idx = this_map_point->observerToIndex[frame.meta.frameID];
                const cv::Point2f &pos = frame.features.positions[corresponding_feature_idx];
                // cout << "the 2D position of this map point on this frame:\n" << pos << endl;
                Eigen::Matrix<double, 2, 1> obs;
                obs << pos.x, pos.y;

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // since only one vertex
                e->setMeasurement(obs);
                float this_keypoint_scale = frame.features.scales[corresponding_feature_idx]; // TODO: change to the actual scale that this keypoint is detected
                e->setInformation(Eigen::Matrix2d::Identity() * (1.0f/this_keypoint_scale));

                g2o::RobustKernelHuber *huber_kernel = new g2o::RobustKernelHuber;
                e->setRobustKernel(huber_kernel);
                huber_kernel->setDelta(THRESH_HUBER);

                // put intrisics to this unary edge
                e->fx = frame.K.at<double>(0,0);
                e->fy = frame.K.at<double>(1,1);
                e->cx = frame.K.at<double>(0,2);
                e->cy = frame.K.at<double>(1,2);

                // put world position to this unary edge
                Point3d point = this_map_point->worldPosition;
                e->Xw[0] = point.x;
                e->Xw[1] = point.y;
                e->Xw[2] = point.z;
                e->setLevel(0); // initially, optimise all

                optimizer.addEdge(e);
                added_edges.push_back(e);
                is_outlier[e] = true; // mark everyone as outlier first
            }
        }
    }

    // if less than 3 correspondence, return, no optimisation done 
    if (num_correspondences < 3) {
        return;
    }

    // start optimise, n iterations, after each round, compute inliers and set flags
    for (int i =0;i < n_round; i++) {
        // TODO: check if use the last round's optimisation result will be better instead of reset every round
        frame_vertex->setEstimate(Converter::cvMatToSE3Quat(frame.Rt)); // reset pose every time
        optimizer.initializeOptimization(0); // optimise on level 0 (inlier level)
        optimizer.optimize(POSE_BA_ITER); // do POSE_BA_ITER number of iterations each round 

        break; // DEBUGGING

        if (i != 0) {
            // classify as inliers and outliers
            for(int j = 0;j< added_edges.size();j++) {
                g2o::EdgeSE3ProjectXYZOnlyPose* this_e = added_edges[j];
                // mark inliers/outliers, will defintely be done when i == 0
                bool is_this_e_outlier = is_outlier[this_e];
                if (is_this_e_outlier) {
                    this_e->computeError();
                    // computes the chi2 based on the cached error value, only valid after computeError has been called, default value = 0
                    float chi2_error = this_e->chi2();
                    if (chi2_error <= CHI2_THRESH) {
                        // mark as inlier
                        is_outlier[this_e] = false;
                        this_e->setLevel(0);
                    }
                    else {
                        // mark as outlier and push to level that will not be optimised
                        is_outlier[this_e] = true;
                        this_e->setLevel(1);
                    }
                }

                if(i == n_round - 2) {
                    this_e->setRobustKernel(NULL); // so that last round, everyone will be inliers
                }
            }

        }
        
    }

    // recover pose optimised
#ifdef DEBUG_POSEBA
    cout << "Rt before :\n" << frame.Rt<< endl;
#endif

    g2o::VertexSE3Expmap * optimised_vertex = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat SE3Quat_pose_optimised = optimised_vertex->estimate();
    frame.Rt = Converter::SE3QuatToCvMat(SE3Quat_pose_optimised);

#ifdef DEBUG_POSEBA
    cout << "Rt after pose optimisation :\n" << frame.Rt<< endl;
#endif
    
}

// Helper function to find frame with target id, return NULL if not found
Frame * Util::findFrameById(vector<Frame> &frames, int target_id) {
    for (int i =0; i < frames.size();i++) {
        if (frames[i].meta.frameID == target_id) {
            return &frames[i];
        }
    }

    return NULL;
}

// Helper function to find Map Point with target id, return NULL if not found
MapPoint* Util::findMapPointById(vector<MapPoint> &map_points, int target_id) {
    for (int i =0; i < map_points.size();i++) {
        if (map_points[i].id == target_id) {
            return &map_points[i];
        }
    }

    return NULL;
}


// Helper function to load frames from .csv files
vector<Frame> Util::loadFrames(string frame_filename) {
    vector<Frame> frames;
	ifstream frame_instream(frame_filename); // ios::binary | ios::ate will just read the size for tellg() to work!
    cout << "frame_filename:" << frame_filename << endl;
	string nextline;
	string token;
    while ( !frame_instream.eof()) {
		// for one frame
        Frame f;        
        cv::Mat Rt = Mat::zeros(3,4, CV_64F);
        
        // read, id, Rt
        getline(frame_instream, nextline);
        cout << "line read in:\n" << nextline << endl;
        if (nextline.length() == 0) {
            continue;
        }
        ostringstream out; // hold space dilimited input
		istringstream is(nextline);
		while (getline(is, token, ',')) {
				out<<token<<" ";
		}
		istringstream is_with_space(out.str());
        is_with_space >> f.meta.frameID ;
        // R
        is_with_space >> Rt.at<double>(0, 0);
        is_with_space >> Rt.at<double>(0, 1);
        is_with_space >> Rt.at<double>(0, 2);
        is_with_space >> Rt.at<double>(1, 0);
        is_with_space >> Rt.at<double>(1, 1);
        is_with_space >> Rt.at<double>(1, 2);
        is_with_space >> Rt.at<double>(2, 0);
        is_with_space >> Rt.at<double>(2, 1);
        is_with_space >> Rt.at<double>(2, 2);
        // t
        is_with_space >> Rt.at<double>(0, 3);
        is_with_space >> Rt.at<double>(1, 3);
        is_with_space >> Rt.at<double>(2, 3);
        f.Rt = Rt;

        cout << "f.Rt:\n" << f.Rt<< endl;
        // read features
        while(getline(frame_instream, nextline) && nextline != "") {
            ostringstream out; // hold space dilimited input
            istringstream is(nextline);
            while (getline(is, token, ',')) {
                    out<<token<<" ";
            }
            istringstream is_with_space(out.str());
            double x, y, sigma;
            is_with_space >> x;
            is_with_space >> y; 
            is_with_space >> sigma;
            f.features.positions.push_back(cv::Point2d(x, y));
            f.features.scales.push_back(sigma);                        
        }

        // read map_points_indices
        while(getline(frame_instream, nextline) && nextline != "") {
            istringstream is(nextline); // nextline will be just a int index
            int this_map_id;
            is >> this_map_id;
            f.features.mapPointsIndices.push_back(this_map_id);
        }

        // add to results
        frames.push_back(f);
    }

    frame_instream.close();
    return frames;
}


// Helper function to load map points from .csv files
vector<MapPoint> Util::loadMapPoints(string map_points_filename) {
    vector<MapPoint> map_points;
	ifstream map_instream(map_points_filename);
	string nextline;
	string token;
    while ( !map_instream.eof()) {
		// for one map point        
        Point3d this_world_pos;
        int id;
        // read, id, world position
        getline(map_instream, nextline);
        if (nextline.length() == 0) {
            continue;// if empty, then skip
        }
        ostringstream out; // hold space dilimited input
		istringstream is(nextline);
		while (getline(is, token, ',')) {
				out<<token<<" ";
		}
		istringstream is_with_space(out.str());
        is_with_space >> id;
        is_with_space >> this_world_pos.x;
        is_with_space >> this_world_pos.y;
        is_with_space >> this_world_pos.z;
        MapPoint this_map_point(this_world_pos, id);

        // read observer to index
        while(getline(map_instream, nextline) && nextline != "") {
            ostringstream out; // hold space dilimited input
            istringstream is(nextline);
            while (getline(is, token, ',')) {
                    out<<token<<" ";
            }
            istringstream is_with_space(out.str());
            int observer_idx, corr_feature_idx;
            is_with_space >> observer_idx;
            is_with_space >> corr_feature_idx;
            this_map_point.addObservingFrame(observer_idx, corr_feature_idx);
        }

        // add to results
        map_points.push_back(this_map_point);
    }

    map_instream.close();
    return map_points;
}

// load only frame id + Optimised Rt from .csv files
vector<Frame> Util::loadFramesOnlyIdRt(string frame_filename) {
    vector<Frame> frames;
	ifstream frame_instream(frame_filename);
    cout << "frame_filename:" << frame_filename << endl; 
	string nextline;
	string token;
    while ( !frame_instream.eof()) {
		// for one frame
        Frame f;        
        cv::Mat Rt = Mat::zeros(3,4, CV_64F);
        
        // read, id, Rt
        getline(frame_instream, nextline);
        cout << "line read:\n" << nextline << endl;
        if (nextline.length() == 0) {
            continue;// if empty, then skip
        }
        ostringstream out; // hold space dilimited input
		istringstream is(nextline);
		while (getline(is, token, ',')) {
				out<<token<<" ";
		}
		istringstream is_with_space(out.str());
        is_with_space >> f.meta.frameID ;
        // R
        is_with_space >> Rt.at<double>(0, 0);
        is_with_space >> Rt.at<double>(0, 1);
        is_with_space >> Rt.at<double>(0, 2);
        is_with_space >> Rt.at<double>(1, 0);
        is_with_space >> Rt.at<double>(1, 1);
        is_with_space >> Rt.at<double>(1, 2);
        is_with_space >> Rt.at<double>(2, 0);
        is_with_space >> Rt.at<double>(2, 1);
        is_with_space >> Rt.at<double>(2, 2);
        // t
        is_with_space >> Rt.at<double>(0, 3);
        is_with_space >> Rt.at<double>(1, 3);
        is_with_space >> Rt.at<double>(2, 3);
        f.Rt = Rt;

        // add to results
        frames.push_back(f);
    }
    frame_instream.close();
    return frames;
}

// load only mappoint id + Optimised XYZ from .csv files
vector<MapPoint> Util::loadMapPointsOnlyIdXYZ(string map_points_filename) {
    vector<MapPoint> map_points;
	ifstream map_instream(map_points_filename);
	string nextline;
	string token;
    while ( !map_instream.eof()) {
		// for one map point        
        Point3d this_world_pos;
        int id;
        // read, id, world position
        getline(map_instream, nextline);
        if (nextline.length() == 0) {
            continue;// if empty, then skip
        }
        ostringstream out; // hold space dilimited input
		istringstream is(nextline);
		while (getline(is, token, ',')) {
				out<<token<<" ";
		}
        
		istringstream is_with_space(out.str());
        is_with_space >> id;
        is_with_space >> this_world_pos.x;
        is_with_space >> this_world_pos.y;
        is_with_space >> this_world_pos.z;
        MapPoint this_map_point(this_world_pos, id);
        
        map_points.push_back(this_map_point);
    }

    map_instream.close();
    return map_points;
}

