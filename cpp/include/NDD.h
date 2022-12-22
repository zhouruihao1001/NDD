#pragma once

// standard lib
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <iostream>

// ext
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// local lib
#include "nanoflann/KDTreeVectorOfVectorsAdaptor.h"
#include "util.h"
#include "tictoc.h"

using NDDPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function.
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = nanoflann::KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

namespace ndd
{

class NDDManager
{
public:
    NDDManager() = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
    
    Eigen::MatrixXd makeNDD(pcl::PointCloud<NDDPointType> &_scan_down);
    
    Eigen::MatrixXd AlignmentKeyFromNDD(Eigen::MatrixXd &_desc);
    
    Eigen::MatrixXd SearchingKeyFromNDD(Eigen::MatrixXd &_desc);
    
    int fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2);
    
    double CorrBtnNDD(Eigen::MatrixXd &_ndd1, Eigen::MatrixXd &_ndd2);
    
    std::pair<double, int> distanceBtnNDD(Eigen::MatrixXd &_ndd1, Eigen::MatrixXd &_ndd2);
    
    // User-side API
    void makeAndSaveNDDAndKeys(pcl::PointCloud<NDDPointType> &_scan_down);
    
    std::tuple<int, int, float> detectLoopClosureID(); // int: nearest node index, float: relative yaw

public:
    
    const int PC_NUM_RING = 20; // 20 rings
    const int PC_NUM_SECTOR = 60; // 60 sectors
    const double PC_MAX_RADIUS = 80.0; // 80 meter max  range
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
    const int max_num_in_cell = 30;//normally 5-50 points in one cell after down-sampling.
    
    // tree
    const size_t NUM_EXCLUDE_RECENT = 100; // TODO 1 for testing, default 100. simply just keyframe gap, but node position distance-based exclusion is ok.
    const int NUM_CANDIDATES_FROM_TREE = 25; // 10-25 is enough.
    
    // loop thres
    const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay.
    const double DIST_THRES = 0.4; // 0.3-0.4 is good choice for hard threshold
    
    // config 
    const int TREE_MAKING_PERIOD_ = 1; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int tree_making_period_conter = 0;
    
    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
    
    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;
    
}; // NDDManagers

} // end namespace ndd

