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

/**
 * @brief Iterates through image directory and begins pipeline of refining feature detections. 
 * 
 * Note that all function names in the pipeline are currently named after 10K param toolbox for standardization, but 
 * there is absolutely no feature detection happening within this pipeline.
 * 
 * @param image_directory 
 * @param detector 
 * @param dataset 
 * @param calibration_window 
 */
void ExtractFeatures(
    std::filesystem::path image_directory,
    FeatureDetector &detector,
    Dataset &dataset,
    CalibrationWindow *calibration_window)
{
    // TODO: read in initial features to be refined, homography.
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
            std::cout << "[ERROR] Cannot read image: " << path << ". Aborting." << std::endl;
            return;
        }
        if (!image_size_set)
            dataset.SetImageSize(0, image.size());
        // I've removed an else statement that checks all imagesizes are the same.

        Image<Vec3u8> detection_visualization;
        vector<Vec2f> features; // TODO: investigate this.
        detector.DetectFeatures(image, features, calibration_window ? &detection_visualization : nullptr); 
        // TODO: Implement DetectFeatures
        std::cout << path << ": " << features.size() << " features" << std::endl;
        calibration_window->UpdateFeatureDetection(0, detection_visualization);
    }
}

/**
 * @brief Entrypoint for main function to create thread.
 * 
 * @param window_half_extent 
 * @param path_to_image 
 * @param image_folder 
 * @param calibration_window 
 */
void refine_features(
    int window_half_extent,
    std::string path_to_image,
    std::string image_folder,
    CalibrationWindow *calibration_window)
{
    Dataset dataset(1);       // Creating a single dataset. We'll assume there's always only one folder to look at for now.
    FeatureDetector detector("/home/siheng/dha/symmetry/input.yaml", window_half_extent, FeatureRefinement::GradientsXY); // TODO: initialize this.

    if (calibration_window)
    {
        calibration_window->SetDataset(&dataset);
    }

    std::filesystem::path fp(path_to_image);
    std::filesystem::path image_fn(image_folder);

    // Iterating through images within specified folder.
    for (auto const &dir_entry : std::filesystem::directory_iterator{fp / image_fn})
    {
        Image<Vec3u8> image;
        Image<u8> gray_image;
        Image<Vec2f> gradient_image;
        Image<float> gradmag_image;

        image.Read(dir_entry.path().string());
        image.ConvertToGrayscale(&gray_image);

        ExtractFeatures(
            std::filesystem::path{fp / image_fn},
            detector,
            dataset,
            calibration_window);
    }
}

int main(int argc, char **argv)
{
    int window_half_extent = 21;
    // "Half size of the search window for pattern corner features. 
    // There is a tradeoff here: on the one hand, it should be as small as possible 
    // to be able to detect features close to the image borders and include little distortion. 
    // On the other hand, it needs to be large enough to be able to detect features properly. 
    // Especially if corners are blurred, it is necessary to increase this extent.");

    bool show_visualizations = true;
    QApplication qapp(argc, argv);
    qapp.setQuitOnLastWindowClosed(false);

    // Create the main window.
    CalibrationWindow calibration_window(nullptr, Qt::WindowFlags());
    if (show_visualizations)
    {
        calibration_window.show();
        calibration_window.raise();
    }

    // Starting a thread for UI to work while feature refinement is underway.
    std::thread calibrate_thread([&]
                                 {
        refine_features(
            window_half_extent,
            std::string("/home/siheng/dha/symmetry"),
            std::string("image"),
            show_visualizations ? &calibration_window : nullptr
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
