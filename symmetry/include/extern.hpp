#include "cpu_refinement_by_symmetry.hpp"
#include "feature_detector.hpp"
#include "puzzlepaint_visualizer/libvis.h"
#include "puzzlepaint_visualizer/dataset.h"
#include "puzzlepaint_visualizer/image_display.h"
#include "puzzlepaint_visualizer/calibration_window.h"
#include <opencv2/core/core.hpp>

// function to use within c++
// returns eigen matrix of predictions, for features that are not recognized please return [-1,-1]
 Eigen::MatrixXd refine_features_bare(
    int window_half_extent,
    int star_segments,
    float star_size, // mm
    cv::Mat image,
    Eigen::MatrixXd initial_prediction, // [px]
    Eigen::MatrixXd stars_target_frame, // [mm]
    std::vector<cv::Mat> homographies  
)
{


}