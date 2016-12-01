//
// Created by JimXing on 12/11/16.
//

#include "Util.h"
#include "Commong2o.h"
#include <map>
#include "Converter.h"
#include "ParamConfig.h"
#include <set>

using namespace std;
using namespace g2o;

typedef BlockSolver< BlockSolverTraits<6, 3> > SLAMBlockSolver;
typedef LinearSolverEigen<SLAMBlockSolver::PoseMatrixType> SLAMLinearSolver;
typedef std::map<Frame*, int>::iterator map_frame_iter;

cv::Mat Util:: ComputeF(Frame f1, Frame f2) {
    return cv::Mat();
}

void Util::GlobalBundleAdjustemnt(DataManager &data, int n_iterations, bool pb_stop_flag,
                                  const unsigned long n_loop_kf, const bool b_robust) {
    // just invoke the usual bundle adjustment function with all map points and key frames maintained in data manager
    Util::BundleAdjustment(data, data.frames,data.mapPoints,n_iterations,pb_stop_flag,n_loop_kf,b_robust);
}

void Util::BundleAdjustment(DataManager &data, vector<Frame> &frames, vector<MapPoint> &map_points,
                             int n_iterations, bool pb_stop_flag, const unsigned long n_loop_kf,
                             const bool b_robust) {
    SparseOptimizer optimizer;
    SLAMLinearSolver *linear_solver = new SLAMLinearSolver();
    linear_solver->setBlockOrdering(false);
    SLAMBlockSolver  *block_solver = new SLAMBlockSolver(linear_solver);

    OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(algo);

    int frame_max_id = 0; // hold the most recent frame id
    set<long unsigned int> frame_vertex_id_set;

    // create vertex for Frames
    for (int i =0;i<frames.size();i++) {
        Frame * this_kf = &frames[i];
        g2o::VertexSE3Expmap * frame_vertex = new g2o::VertexSE3Expmap();

        frame_vertex->setEstimate(Converter::cvMatToSE3Quat(this_kf->Rt));
        frame_vertex->setId(this_kf->meta.frameID);
        frame_vertex->setFixed(this_kf->meta.frameID ==0); // fix the first frame
        optimizer.addVertex(frame_vertex);
        if(this_kf->meta.frameID > frame_max_id)
            frame_max_id=this_kf->meta.frameID;
        frame_vertex_id_set.insert(this_kf->meta.frameID);
    }

    int optimized[map_points.size()];
    // create vertex for MapPoints
    for (int i =0;i<map_points.size();i++) {
        MapPoint *this_map_point = &map_points[i];
        map<Frame *, int> observations = this_map_point->observer_to_index; // get all observers mapping

        map_frame_iter iterator;
        // check if it does not have associated frame in the frame_vertex_id_set
        for (iterator = observations.begin(); iterator != observations.end(); iterator++) {
            Frame *observer = iterator->first;
            if (frame_vertex_id_set.find(observer->meta.frameID) != frame_vertex_id_set.end()) {
                break;
            }
        }
        // ignore this mapPoint, if no associsations with frame inserted to optimizer
        if (iterator == observations.end()) {
            optimized[i] = false;
            continue;
        }
        optimized[i] = true;

        g2o::VertexSBAPointXYZ *point_vertex = new g2o::VertexSBAPointXYZ();

        point_vertex->setEstimate(Converter::cvMatToVector3d(this_map_point->world_pos));
        // id for point vertex, starting from frame_max_id + 1 to not overlap with frame_vertex ids
        int map_point_id = this_map_point->id + frame_max_id + 1;
        point_vertex->setId(map_point_id);
        point_vertex->setMarginalized(true);
        optimizer.addVertex(point_vertex);

        // for each added MapPoint, create edge from its observers
        for (map_frame_iter iterator = observations.begin(); iterator != observations.end(); iterator++) {

            // iterator->first = Frame *
            // iterator->second = int
            Frame *observer = iterator->first;
            int feature_idx = iterator->second;
            // ignore if the observer is not in frame vertexes added
            if (frame_vertex_id_set.find(observer->meta.frameID) != frame_vertex_id_set.end()) {

                const cv::Point2f &pos = observer->features.positions[i];

                Eigen::Matrix<double, 2, 1> obs;
                obs << pos.x, pos.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(map_point_id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(observer->meta.frameID)));
                e->setMeasurement(obs);
                float this_keypoint_scale = 1.0; // TODO: change to the actual scale that this keypoint is detected
                e->setInformation(Eigen::Matrix2d::Identity() * this_keypoint_scale);

                if (b_robust) {
                    g2o::RobustKernelHuber *huber_kernel = new g2o::RobustKernelHuber;
                    e->setRobustKernel(huber_kernel);
                    huber_kernel->setDelta(THRESH_HUBER);
                }

                e->fx = data.camera_intrinsics.at<double>(0,0);
                e->fy = data.camera_intrinsics.at<double>(1,1);
                e->cx = data.camera_intrinsics.at<double>(0,2);
                e->cy = data.camera_intrinsics.at<double>(1,2);

                optimizer.addEdge(e);
            }
        }
    }
    // end of construction of graph for BA

    // start optimisation
    optimizer.initializeOptimization();
    optimizer.optimize(n_iterations);

    // Recover optimized data

    //Keyframes
    for(int i=0; i<frames.size(); i++) {
        Frame* this_frame = &frames[i];
        g2o::VertexSE3Expmap* frame_vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(this_frame->meta.frameID));
        g2o::SE3Quat SE3quat = frame_vertex->estimate();
        if(n_loop_kf==0) {
            this_frame->Rt = Converter::SE3QuatToCvMat(SE3quat);
        }
        // TODO: check if needs to maintain different poses for n_loop_kf != 0, loop points
    }

    //Map Points
    for(int i=0; i<map_points.size(); i++) {
        if(optimized[i]) {
            MapPoint* this_map_point = &map_points[i];

            g2o::VertexSBAPointXYZ* point_vertex = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(this_map_point->id+frame_max_id+1));

            if(n_loop_kf==0) {
                this_map_point->world_pos = Converter::vector3DToCvMat(point_vertex->estimate());
            }
            // TODO: check if needs to maintain different 3D coords for n_loop_kf != 0, loop points
        }
    }
}

void Util::PoseBundleAdjustment(Frame &frame, DataManager &data, int n_round) {
    SparseOptimizer optimizer;
    SLAMLinearSolver *linear_solver = new SLAMLinearSolver();
    linear_solver->setBlockOrdering(false);
    SLAMBlockSolver  *block_solver = new SLAMBlockSolver(linear_solver);

    OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(algo);

    // insert the frame vertex
    g2o::VertexSE3Expmap * frame_vertex = new g2o::VertexSE3Expmap();
    frame_vertex->setEstimate(Converter::cvMatToSE3Quat(frame.Rt));
    frame_vertex->setId(0); // since only one frame here
    frame_vertex->setFixed(false);
    optimizer.addVertex(frame_vertex);

    int num_map_points = frame.map_points.size();
    int num_correspondences = 0;
    // keep a reference to the added edges
    vector<g2o::EdgeSE3ProjectXYZOnlyPose * > added_edges;
    map<g2o::EdgeSE3ProjectXYZOnlyPose *, bool> is_outlier;

    // insert MapPoint as EdgeSE3ProjectXYZOnlyPose
    for (int i =0;i < num_map_points;i++) {
        MapPoint * this_map_point = frame.map_points[i];
        if (this_map_point != NULL) {
            g2o::EdgeSE3ProjectXYZOnlyPose * e = new g2o::EdgeSE3ProjectXYZOnlyPose();
            num_correspondences ++;
            
            int corresponding_feature_idx = this_map_point->observer_to_index[&frame];
            const cv::Point2f &pos = frame.features.positions[corresponding_feature_idx];
            Eigen::Matrix<double, 2, 1> obs;
            obs << pos.x, pos.y;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // since only one vertex
            e->setMeasurement(obs);
            float this_keypoint_scale = 1.0; // TODO: change to the actual scale that this keypoint is detected
            e->setInformation(Eigen::Matrix2d::Identity() * this_keypoint_scale);


            g2o::RobustKernelHuber *huber_kernel = new g2o::RobustKernelHuber;
            e->setRobustKernel(huber_kernel);
            huber_kernel->setDelta(THRESH_HUBER);

            // put intrisics to this unary edge
            e->fx = data.camera_intrinsics.at<double>(0,0);
            e->fy = data.camera_intrinsics.at<double>(1,1);
            e->cx = data.camera_intrinsics.at<double>(0,2);
            e->cy = data.camera_intrinsics.at<double>(1,2);

            // put world position to this unary edge
            cv::Mat Xw = this_map_point->world_pos;
            e->Xw[0] = Xw.at<float>(0);
            e->Xw[1] = Xw.at<float>(1);
            e->Xw[2] = Xw.at<float>(2);
            e->setLevel(0); // initially, optimise all

            optimizer.addEdge(e);
            added_edges.push_back(e);
            is_outlier[e] = true; // mark everyone as outlier first
        }
    }

    // if less than 3 correspondence, return, no optimisation done 
    if (num_correspondences < 3) {
        return;
    }

    // start optimise, n iterations, after each round, compute inliers and set flags
    for (int i =0;i < n_round; i++) {
        // TODO: check if use the last round's optimisation result will be better
        frame_vertex->setEstimate(Converter::cvMatToSE3Quat(frame.Rt)); // reset pose every time
        optimizer.initializeOptimization(0); // optimise on level 0 (inlier level)
        optimizer.optimize(POSE_BA_ITER); // do POSE_BA_ITER number of iterations each round 

        // classify as inliers and outliers
        for(int j = 0;j< added_edges.size();j++ )
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = added_edges[j];
            // mark inliers/outliers, will defintely be done when i == 0
            bool is_this_e_outlier = is_outlier[e];
            if (is_this_e_outlier) {
                e->computeError();
                // computes the chi2 based on the cached error value, only valid after computeError has been called, default value = 0
                float chi2_error = e->chi2();
                if (chi2_error <= CHI2_THRESH) {
                    // mark as inlier
                    is_outlier[e] = false;
                    e->setLevel(0);
                }
                else {
                    // mark as outlier and push to level that will not be optimised
                    is_outlier[e] = true;
                    e->setLevel(1);
                }
            }

            if(i == n_round - 2) {
                e->setRobustKernel(NULL); // so that last round, everyone will be inliers
            }
        }
    }

    // recover pose optimised
    g2o::VertexSE3Expmap * optimised_vertex = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat SE3Quat_pose_optimised = optimised_vertex->estimate();
    frame.Rt = Converter::SE3QuatToCvMat(SE3Quat_pose_optimised);;
    
}