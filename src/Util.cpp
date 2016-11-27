//
// Created by JimXing on 12/11/16.
//

#include "Util.h"
#include "Commong2o.h"
#include <map>
#include "Converter.h"
#include <set>

using namespace std;
using namespace g2o;

typedef BlockSolver< BlockSolverTraits<6, 3> > SLAMBlockSolver;
typedef LinearSolverEigen<SLAMBlockSolver::PoseMatrixType> SLAMLinearSolver;
typedef std::map<KeyFrame*, size_t>::iterator map_keyframe_iter;

cv::Mat Util:: ComputeF(Frame f1, Frame f2) {
    return cv::Mat();
}

void Util::GlobalBundleAdjustemnt(DataManager &data, int n_iterations, bool *pb_stop_flag,
                                  const unsigned long n_loop_kf, const bool b_robust) {
    // just invoke the usual bundle adjustment function with all map points and key frames maintained in data manager
    Util::BundleAdjustment(data.all_keyframes,data.all_map_points,n_iterations,pb_stop_flag,n_loop_kf,b_robust);
}

void Util::BundleAdjustment(const std::vector<KeyFrame*> &keyframes, const std::vector<MapPoint*> &map_points,
                             int n_iterations, bool *pb_stop_flag, const unsigned long n_loop_kf,
                             const bool b_robust) {
    SparseOptimizer optimizer;
    SLAMLinearSolver *linear_solver = new SLAMLinearSolver();
    linear_solver->setBlockOrdering(false);
    SLAMBlockSolver  *block_solver = new SLAMBlockSolver(linear_solver);

    OptimizationAlgorithmLevenberg* algo = new OptimizationAlgorithmLevenberg(block_solver);
    optimizer.setAlgorithm(algo);

    int key_frame_max_id = 0; // hold the most recent keyframe id
    set<long unsigned int> keyframe_vertex_id_set;

    const float thresh_huber = sqrt(5.99); // adjustable parameters for huber cost function

    // create vertex for KeyFrames
    for (int i =0;i<keyframes.size();i++) {
        KeyFrame * this_kf = keyframes[i];
        g2o::VertexSE3Expmap * keyframe_vertex = new g2o::VertexSE3Expmap();

        keyframe_vertex->setEstimate(Converter::cvMatToSE3Quat(this_kf->Rt));
        keyframe_vertex->setId(this_kf->id);
        keyframe_vertex->setFixed(this_kf->id ==0);
        optimizer.addVertex(keyframe_vertex);
        if(this_kf->id > key_frame_max_id)
            key_frame_max_id=this_kf->id;
        keyframe_vertex_id_set.insert(this_kf->id);
    }

    int optimized[map_points.size()];
    // create vertex for MapPoints
    for (int i =0;i<map_points.size();i++) {
        MapPoint *this_map_point = map_points[i];
        map<KeyFrame *, size_t> observations = this_map_point->observer_to_index; // get all observers mapping

        map_keyframe_iter iterator;
        // check if it does not have associated keyframe in the keyframe_vertex_id_set
        for (iterator = observations.begin(); iterator != observations.end(); iterator++) {
            KeyFrame *observer = iterator->first;
            if (!(keyframe_vertex_id_set.find(observer->id) == keyframe_vertex_id_set.end())) {
                break;
            }
        }
        // ignore this mapPoint, if no associsations with keyframe inserted to optimizer
        if (iterator == observations.end()) {
            optimized[i] = false;
            continue;
        }
        optimized[i] = true;

        g2o::VertexSBAPointXYZ *point_vertex = new g2o::VertexSBAPointXYZ();

        point_vertex->setEstimate(Converter::cvMatToVector3d(this_map_point->world_pos));
        // id for point vertex, starting from key_frame_max_id + 1 to not overlap with keyframe_vertex ids
        int map_point_id = this_map_point->id + key_frame_max_id + 1;
        point_vertex->setId(map_point_id);
        point_vertex->setMarginalized(true);
        optimizer.addVertex(point_vertex);

        // for each added MapPoint, create edge from its observers
        for (map_keyframe_iter iterator = observations.begin(); iterator != observations.end(); iterator++) {

            // iterator->first = KeyFrame *
            // iterator->second = size_t
            KeyFrame *observer = iterator->first;
            size_t feature_idx = iterator->second;
            // ignore if the observer is not in keyframe vertexes added
            if (!(keyframe_vertex_id_set.find(observer->id) == keyframe_vertex_id_set.end())) {

                const cv::Point2f &pos = observer->features.positions[i];

                Eigen::Matrix<double, 2, 1> obs;
                obs << pos.x, pos.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(map_point_id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(observer->id)));
                e->setMeasurement(obs);
                float this_keypoint_scale = 1.0; // TODO: change to the actual scale that this keypoint is detected
                e->setInformation(Eigen::Matrix2d::Identity() * this_keypoint_scale);

                if (b_robust) {
                    g2o::RobustKernelHuber *huber_kernel = new g2o::RobustKernelHuber;
                    e->setRobustKernel(huber_kernel);
                    huber_kernel->setDelta(thresh_huber);
                }

//                e->fx = observer->fx;
//                e->fy = observer->fy;
//                e->cx = observer->cx;
//                e->cy = observer->cy;

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
    for(int i=0; i<keyframes.size(); i++) {
        KeyFrame* this_keyframe = keyframes[i];
        g2o::VertexSE3Expmap* keyframe_vertex = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(this_keyframe->id));
        g2o::SE3Quat SE3quat = keyframe_vertex->estimate();
        if(n_loop_kf==0) {
            this_keyframe->Rt = Converter::SE3QuatToCvMat(SE3quat);
        }
        // TODO: check if needs to maintain different poses for n_loop_kf != 0, loop points
    }

    //Map Points
    for(int i=0; i<map_points.size(); i++) {
        if(optimized[i]) {
            MapPoint* this_map_point = map_points[i];

            g2o::VertexSBAPointXYZ* point_vertex = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(this_map_point->id+key_frame_max_id+1));

            if(n_loop_kf==0) {
                this_map_point->world_pos = Converter::vector3DToCvMat(point_vertex->estimate());
            }
            // TODO: check if needs to maintain different 3D coords for n_loop_kf != 0, loop points
        }
    }
}