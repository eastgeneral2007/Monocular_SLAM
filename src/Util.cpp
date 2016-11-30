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
typedef std::map<Frame*, size_t>::iterator map_frame_iter;

cv::Mat Util:: ComputeF(Frame f1, Frame f2) {
    return cv::Mat();
}

void Util::GlobalBundleAdjustemnt(DataManager &data, int n_iterations, bool pb_stop_flag,
                                  const unsigned long n_loop_kf, const bool b_robust) {
    // just invoke the usual bundle adjustment function with all map points and key frames maintained in data manager
    Util::BundleAdjustment(data.frames,data.mapPoints,n_iterations,pb_stop_flag,n_loop_kf,b_robust);
}

void Util::BundleAdjustment(vector<Frame> &frames, vector<MapPoint> &map_points,
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

    const float thresh_huber = sqrt(5.99); // adjustable parameters for huber cost function

    // create vertex for Frames
    for (int i =0;i<frames.size();i++) {
        Frame * this_kf = &frames[i];
        g2o::VertexSE3Expmap * frame_vertex = new g2o::VertexSE3Expmap();

        frame_vertex->setEstimate(Converter::cvMatToSE3Quat(this_kf->Rt));
        frame_vertex->setId(this_kf->meta.frameID);
        frame_vertex->setFixed(this_kf->meta.frameID ==0);
        optimizer.addVertex(frame_vertex);
        if(this_kf->meta.frameID > frame_max_id)
            frame_max_id=this_kf->meta.frameID;
        frame_vertex_id_set.insert(this_kf->meta.frameID);
    }

    int optimized[map_points.size()];
    // create vertex for MapPoints
    for (int i =0;i<map_points.size();i++) {
        MapPoint *this_map_point = &map_points[i];
        map<Frame *, size_t> observations = this_map_point->observer_to_index; // get all observers mapping

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
            // iterator->second = size_t
            Frame *observer = iterator->first;
            size_t feature_idx = iterator->second;
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