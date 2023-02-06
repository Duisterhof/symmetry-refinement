#include <iostream>
#include <filesystem> // directory_iterator
#include <memory>     // unique_ptr
#include <thread>     // thread
#include "cpu_refinement_by_symmetry.hpp"
#include "feature_detector.hpp"

#include "puzzlepaint_visualizer/libvis.h"
#include "puzzlepaint_visualizer/dataset.h"
#include "puzzlepaint_visualizer/image_display.h"
#include "puzzlepaint_visualizer/calibration_window.h"
#include <unistd.h>

using namespace vis;

void ExtractFeatures(std::filesystem::path image_directory, FeatureDetector &detector, Dataset &dataset, CalibrationWindow &calibration_window)
{
    // Filenames of all images with features to be refined.
    std::vector<std::string> filenames;

    // Populating filenames with image files to be worked on.
    for (auto const &dir_entry : std::filesystem::directory_iterator(image_directory))
        filenames.push_back(dir_entry.path().string());
    sort(filenames.begin(), filenames.end());

    bool image_size_set = false;
    for (int imageset_index = 0; imageset_index < filenames.size(); ++imageset_index)
    {
        std::string path = filenames[imageset_index];
        Image<Vec3u8> image(path);
        if (image.empty())
        {
            std::cout << "[ERROR] Cannot read image: " << path << ". Aborting.";
            return;
        }
        if (!image_size_set)
            dataset.SetImageSize(0, image.size());
        // I've removed an else statement that checks all imagesizes are the same.

        Image<Vec3u8> detection_visualization;
        vector<PointFeature> features;
        detector.DetectFeatures(image, features, detection_visualization); // TODO: Implement DetectFeatures
        std::cout << path << ": " << features.size() << " features";
        calibration_window.UpdateFeatureDetection(0, detection_visualization);
    }
}

void refine_features(std::string path_to_image, std::string image_folder, CalibrationWindow &calibration_window)
{
    Dataset dataset(1);       // Creating a single dataset. We'll assume there's always only one folder to look at for now.
    FeatureDetector detector; // TODO: initialize this.

    calibration_window.SetDataset(&dataset);

    FeatureDetector fd;
    std::filesystem::path fp(path_to_image);
    std::filesystem::path image_fn(image_folder);

    // TODO: Implement ExtractFeatures
    ExtractFeatures(
        std::filesystem::path{fp / image_fn},
        detector, // TODO
        dataset,
        calibration_window);

    // Iterating through images within specified folder.
    for (auto const &dir_entry : std::filesystem::directory_iterator{fp / image_fn})
    {
        Image<Vec3u8> image;
        Image<u8> gray_image;
        Image<Vec2f> gradient_image;
        Image<float> gradmag_image;

        image.Read(dir_entry.path().string());
        image.ConvertToGrayscale(&gray_image);
        fd.computeGradientGradmagImages(gray_image, gradient_image, gradmag_image);

        ImageDisplay display;
        display.Clear();
        display.Update(gray_image, "Grayscale");
        std::cout << "Image done." << std::endl;
    }
}

int main(int argc, char **argv)
{
    bool show_visualizations = true;
    QApplication qapp(argc, argv);
    qapp.setQuitOnLastWindowClosed(false);

    // Create the main window.
    CalibrationWindow calibration_window(nullptr, Qt::WindowFlags());
    calibration_window.show();
    calibration_window.raise();

    // Starting a thread for UI to work while feature refinement is underway.
    std::thread calibrate_thread([&]
                                 {
        refine_features(
            std::string("/home/siheng/dha/symmetry"),
            std::string("image"),
            calibration_window
        );

        RunInQtThreadBlocking([&]() {
        if (calibration_window.isVisible()) {
            qapp.setQuitOnLastWindowClosed(true);
        } else {
            qapp.quit();
        }
        }); });

    // Run the Qt event loop
    qapp.exec();

    calibrate_thread.join();
    return EXIT_SUCCESS;

    return 0;
}
