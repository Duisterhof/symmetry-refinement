#include "puzzlepaint_visualizer/eigen.h"
#include "puzzlepaint_visualizer/image.h"
#include "puzzlepaint_visualizer/dataset.h"
#include "puzzlepaint_visualizer/image_display.h"
#include "puzzlepaint_visualizer/hash_vec2i.h"

#include "yaml-cpp/yaml.h"
#include "cpu_refinement_by_matching.hpp"
#include "cpu_refinement_by_symmetry.hpp"

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#ifndef FEATURE_DETECTOR_H_
#define FEATURE_DETECTOR_H_
namespace vis
{
    // Types of FeatureRefinement methods
    enum class FeatureRefinement
    {
        GradientsXY = 0,
        GradientMagnitude,
        Intensities,
        NoRefinement
    };

    /// Data of a single detected feature.
    struct FeatureDetection
    {
        /// Subpixel position in the image ("pixel center" coordinate convention)
        Vec2f position;

        // Integer coordinate of the feature in the pattern
        Vec2i pattern_coordinate;

        // The direction that was used from an older existing feature detection to
        // arrive at this feature detection. May be (0, 0) if the feature was
        // predicted based on an AprilTag. May be uninitialized for final feature
        // detections; this is used for feature predictions only.
        Vec2i extension_direction;

        /// Final cost value of the feature refinement optimization process. This
        /// may be used for outlier detection.
        float final_cost;

        /// Number of times this feature has been validated.
        // int num_validations = 0;  // NOTE: Currently unused

        /// Homography on local coordinate systems. Is supposed to map the integer
        /// feature coordinate in the pattern (where (0, 0) is defined to be the
        /// feature corresponding to this detection in the local coordinate system) to
        /// the pixel position (where pixel (0, 0) is defined to be at the predicted
        /// feature position).
        Mat3f local_pixel_tr_pattern;
    };

    /// Characteristics of a known pattern that can be detected by the detector.
    struct PatternData
    {
        // Returns whether the given integer coordinate is a valid feature coordinate of the pattern.
        // Amended to remove AprilTag check (N/A to customized board.)
        inline bool IsValidFeatureCoord(int x, int y) const
        {
            if (!(x >= 0 && y >= 0 && x <= squares_x - 2 && y <= squares_y - 2))
                return false;
            return true;
        }

        /// Returns whether the floating-point coordinate in the feature coordinate
        /// system is within the area of the calibration target that is covered by
        /// the repeating pattern.
        inline bool IsValidPatternCoord(float x, float y) const
        {
            if (!(x >= -1.f && y >= -1.f && x <= squares_x - 1.f && y <= squares_y - 1.f))
                return false;
            return true;
        }

        /// Returns the pattern intensity (0 for black, 1 for white, 0.5 for ill-defined
        /// positions) at the given position within the pattern. The pattern is supposed
        /// to have endless extent, feature positions are at integer coordinates, and
        /// (0, 0) is supposed to correspond to a feature location.
        template <typename Derived>
        inline float PatternIntensityAt(const MatrixBase<Derived> &position) const
        {
            // Have coordinates in [-0.5, 0.5].
            Vec2f c;
            c.x() = position.x() - (position.x() > 0 ? 1 : -1) * static_cast<int>(std::fabs(position.x()) + 0.5f);
            c.y() = position.y() - (position.y() > 0 ? 1 : -1) * static_cast<int>(std::fabs(position.y()) + 0.5f);

            if (c.squaredNorm() < 1e-8f)
                return 0.5f;

            float angle = std::atan2(c.y(), c.x()) - 0.5f * M_PI;
            if (angle < 0)
                angle += 2 * M_PI;
            return (static_cast<int>(num_star_segments * angle / (2 * M_PI)) % 2 == 0) ? 1.f : 0.f;
        }

        /// Returns a corner position of the given star triangle.
        Vec2f GetStarCoord(float square_length, float i, float center_x, float center_y) const
        {
            float angle = ((2 * M_PI) * i) / num_star_segments;
            float x = sin(angle);
            float y = cos(angle);

            float max_abs_x = max(abs(x), abs(y));
            x /= max_abs_x;
            y /= max_abs_x;

            return Vec2f(center_x - 0.5 * square_length * x,
                         center_y - 0.5 * square_length * y);
        }

        /// Number of squares in x direction.
        /// TODO: The term "square" is probably not applicable anymore to the star pattern?
        int squares_x;

        /// Number of squares in y direction.
        int squares_y;

        /// Number of segments (both black and white segments are counted) in each
        /// "star" around each feature. For example, for a checkerboard this would
        /// be 4.
        int num_star_segments;

        /// Page properties. These are not relevant for feature detection, but are
        /// helpful if one wants to render the pattern synthetically.
        float page_width_mm;
        float page_height_mm;
        float pattern_start_x_mm;
        float pattern_start_y_mm;
        float pattern_end_x_mm;
        float pattern_end_y_mm;
    };

    struct FeatureDetectorTaggedPatternPrivate
    {
        FeatureDetectorTaggedPatternPrivate() = default;

        ~FeatureDetectorTaggedPatternPrivate() {}

        int cost_buffer_size = 0;

        vector<Vec2f> samples; // homogeneously distributed in [-1, 1] x [-1, 1].

        /// Attention, the index of the object in this vector may be different from
        /// its pattern_index. Only the latter is relevant.
        vector<PatternData> patterns;

        int timing_output_counter = 0;
        unique_ptr<ImageDisplay> debug_display;
    };

    class FeatureDetector
    {
    public:
        FeatureDetector(
            const std::string &pattern_yaml_paths,
            int window_half_extent,
            FeatureRefinement refinement_type);

        void RefineFeatureDetections(
            const Image<u8> &image,
            const Image<Vec2f> &gradient_image,
            const Image<float> &gradmag_image,
            int num_features,
            const FeatureDetection *predicted_features,
            FeatureDetection *output,
            bool debug,
            bool debug_step_by_step);

        void computeGradientGradmagImages(
            const Image<u8> &image,
            Image<Vec2f> &gradient_image,
            Image<float> &gradmag_image);

        void DetectFeatures(
            const Image<Vec3u8> &image,
            std::vector<Vec2f> &features,
            const std::string yaml_filename,
            Image<Vec3u8> *detection_Visualization);

        void PredictAndDetectFeatures(
            const Image<u8> &image,
            const Image<Vec2f> &gradient_image,
            const Image<float> &gradmag_image,
            vector<FeatureDetection> *feature_predictions,
            vector<FeatureDetection> *feature_detections,
            bool debug,
            bool debug_step_by_step,
            Vec3u8 debug_colors[8]);

        bool SetPatternYAMLPaths(const std::string path);

        void GetFeaturePredictions(std::vector<FeatureDetection>& feature_predictions);

    private:
        int window_half_extent;
        unique_ptr<FeatureDetectorTaggedPatternPrivate> d;
        FeatureRefinement refinement_type;
        std::string image_folder_name;
        std::string feat_yaml_name;
        std::vector<PatternData> patterns;
        float cell_length_in_meters;
        bool valid_;
    };
}
#endif