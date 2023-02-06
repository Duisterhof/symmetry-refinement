#include "feature_detector.hpp"

namespace vis
{
    void FeatureDetector::PredictAndDetectFeatures(
        const Image<u8> &image,
        const Image<Vec2f> &gradient_image,
        const Image<float> &gradmag_image,
        vector<unordered_map<Vec2i, FeatureDetection>> *feature_predictions,
        vector<unordered_map<Vec2i, FeatureDetection>> *feature_detections,
        vector<unordered_set<Vec2i>> *feature_rejections,
        bool debug,
        bool debug_step_by_step,
        Vec3u8 debug_colors[8])
    {
        const int kIncrementalPredictionErrorThreshold = window_half_extent * 4 / 5.f; // In pixels. TODO: make configurable
        constexpr float kMinimumFeatureDistance = 5;                                   // in pixels; TODO: make configurable

        int num_predictions = 0;
        for (auto &pattern_feature_predictions : *feature_predictions)
            num_predictions += pattern_feature_predictions.size();

        vector<bool> discard_pattern_detection(feature_predictions->size(), false);
        feature_detections->resize(feature_predictions->size());
        feature_rejections->resize(feature_predictions->size());

    }

    void FeatureDetector::RefineFeatureDetections(
        const Image<u8> &image,
        const Image<Vec2f> &gradient_image,
        const Image<float> &gradmag_image,
        int num_features,
        const FeatureDetection *output,
        bool debug)
    {
    }

    void FeatureDetector::computeGradientGradmagImages(
        const Image<u8> &image,
        Image<Vec2f> &gradient_image,
        Image<float> &gradmag_image)
    {
        gradient_image.SetSize(image.size());
        gradmag_image.SetSize(image.size());
        int width = image.width();
        int height = image.height();

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int mx = std::max<int>(0, x - 1);
                int px = std::min<int>(width - 1, x + 1);

                int my = std::max<int>(0, y - 1);
                int py = std::min<int>(height - 1, y + 1);

                float dx = (image(px, y) - static_cast<float>(image(mx, y))) / (px - mx);
                float dy = (image(x, py) - static_cast<float>(image(x, my))) / (py - my);

                gradient_image(x, y) = Vec2f(dx, dy);
                gradmag_image(x, y) = gradient_image(x, y).norm();
            }
        }
    }

    void FeatureDetector::DetectFeatures(
        const Image<Vec3u8> &image,
        std::vector<PointFeature> &features,
        Image<Vec3u8> &detection_visualization)
    {
        // Setting up image to be visualized
        detection_visualization.SetSize(image.size());
        detection_visualization.SetTo(image);

        // Prepare sample positions.
        int max_sample_count = static_cast<int>(8.0 * (2 * window_half_extent + 1) * (2 * window_half_extent + 1) + 0.5);
        if (d->samples.empty() ||
            d->samples.size() < max_sample_count)
        {
            d->samples.resize(max_sample_count);
            srand(0);
            for (usize i = 0; i < d->samples.size(); ++i)
            {
                d->samples[i] = Vec2f::Random();
            }
        }

        // Convert the image to grayscale.
        Image<u8> gray_image;
        image.ConvertToGrayscale(&gray_image);

        // Compute gradient image.
        Image<Vec2f> gradient_image;
        Image<float> gradmag_image;

        gradient_image.SetSize(gray_image.size());
        gradmag_image.SetSize(gray_image.size());
        int width = gray_image.width();
        int height = gray_image.height();
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int mx = std::max<int>(0, x - 1);
                int px = std::min<int>(width - 1, x + 1);

                int my = std::max<int>(0, y - 1);
                int py = std::min<int>(height - 1, y + 1);

                float dx = (gray_image(px, y) - static_cast<float>(gray_image(mx, y))) / (px - mx);
                float dy = (gray_image(x, py) - static_cast<float>(gray_image(x, my))) / (py - my);

                gradient_image(x, y) = Vec2f(dx, dy);
                gradmag_image(x, y) = gradient_image(x, y).norm();
            }
        }

        static Vec3u8 debug_colors[8] = {
            Vec3u8(255, 80, 80),
            Vec3u8(255, 80, 255),
            Vec3u8(80, 255, 255),
            Vec3u8(0, 255, 0),
            Vec3u8(80, 80, 255),
            Vec3u8(127, 255, 127),
            Vec3u8(255, 160, 0),
            Vec3u8(255, 255, 0)};

        vector<unordered_map<Vec2i, FeatureDetection>> feature_predictions; // TODO: Read features into this format.

        /// Mapping: feature_predictions[pattern_array_index][Vec2i(pattern_x, pattern_y)] -> final feature detection.
        vector<unordered_map<Vec2i, FeatureDetection>> feature_detections;

        /// Contains rejected detections: feature_rejections[pattern_array_index][Vec2i(pattern_x, pattern_y)]
        vector<unordered_set<Vec2i>> feature_rejections;

        PredictAndDetectFeatures(
            gray_image,
            gradient_image,
            gradmag_image,
            &feature_predictions,
            &feature_detections,
            &feature_rejections,
            false,
            false,
            debug_colors);
    }
}