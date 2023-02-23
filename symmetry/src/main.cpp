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

#include <algorithm> // for command line parsing

#include "CLI11.hpp"

using namespace vis;

/**
 * @brief Entrypoint for main function to create thread. (Replaces ExtractFeatures in 10K params)
 * 
 * @param window_half_extent window to sample from (used by both intensity-based / symmetry-based refinement)
 * @param path_to_image path to image file
 * @param image_folder folder containing images
 * @param calibration_window 
 */
void refine_features(
    int window_half_extent,
    std::string path_to_image,
    std::string path_to_predictions,
    std::string path_to_results,
    CalibrationWindow *calibration_window)
{
    const std::string IMAGE_FORMAT(".png"); // USED as a simple check for image format.

    Dataset dataset(1);// Creating a single dataset. We'll assume there's always only one folder to look at for now.

    if (calibration_window)
        calibration_window->SetDataset(&dataset);

    // For reading.
    std::filesystem::path image_fp(path_to_image);
    std::filesystem::path pred_fp(path_to_predictions);

    // Filenames of all images with features to be refined.
    std::vector<std::string> filenames;

    // Populating filenames with image files to be worked on.
    for (auto const &dir_entry : std::filesystem::directory_iterator(image_fp)) {
        std::string current_file = dir_entry.path().string();
        bool invalid_image = true;

        // Performs a quick check to ensure that image format is correct.
        if (current_file.length() >= IMAGE_FORMAT.length()) {
            invalid_image = current_file.compare(
                    current_file.length() - IMAGE_FORMAT.length(),
                    IMAGE_FORMAT.length(), 
                    IMAGE_FORMAT);
        }
        
        // If image format is valid.
        if (!invalid_image)
            filenames.push_back(dir_entry.path().string());
    }
    
    // Sorting filenames into order
    sort(filenames.begin(), filenames.end());
    bool image_size_set = false;
    // Iterating through all valid images and refining features within them
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

        // Getting image name without an extension to find correct YAML file.
        string image_file = std::filesystem::path(path).filename().string();
        size_t lastindex = image_file.find_last_of("."); 
        string image_name = image_file.substr(0, lastindex);

        // Finding correct prediction yaml by getting the filename part of each (removing extension)
        // i.e. pred.yaml = pred
        // and comparing it to the image filename in a similar way.
        std::string yaml_name;
        std::string current_yaml_filepath;
        bool found_yaml = false;
        for (auto const &dir_entry : std::filesystem::directory_iterator(pred_fp)) {
            // Getting the filename part of each predictions yaml and
            current_yaml_filepath = dir_entry.path().string();
            std::string current_file = dir_entry.path().filename().string();
            lastindex = current_file.find_last_of("."); 
            yaml_name = current_file.substr(0, lastindex);

            std::cout << "image_name:" << image_name << std::endl;
            std::cout << "yaml_name:" << yaml_name << std::endl;
            std::cout << "files are same:" << yaml_name.compare(image_name) << std::endl;

            // same!
            if (yaml_name.compare(image_name) == 0) {
                found_yaml = true;
                break;
            }
        }

        if (!found_yaml) {
            std::cout << "[ERROR] Missing predictions file for image_name = " << image_name;
            continue;
        }

        std::cout << "[INFO] Found predictions YAML file:" << current_yaml_filepath << std::endl;
        std::string results_filepath = path_to_results + "/" + yaml_name + ".csv";
        std::cout << "[INFO] Output results to: " << results_filepath << std::endl;
        // Initialize new 10k param detector for feature refinement.
        FeatureDetector detector(
            current_yaml_filepath,
            window_half_extent,
            FeatureRefinement::GradientsXY
        );

        Image<Vec3u8> detection_visualization;
        vector<pair<bool, Vec2f>> features;
        detector.DetectFeatures(image, features, calibration_window ? &detection_visualization : nullptr); 
        
        // Debugging main output from feature refinement
        // TODO: write to CSV.
        std::cout << path << ": " << features.size() << " features" << std::endl;
        
        std::ofstream results_file;
        results_file.open(results_filepath);
        for (auto& refinement : features) {
            results_file << refinement.first << "," << refinement.second.x() << "," << refinement.second.y() << std::endl;
        }
        results_file.close();

        if (calibration_window)
            calibration_window->UpdateFeatureDetection(0, detection_visualization);
    }
}

int main(int argc, char **argv)
{
    CLI::App app{"Symmetry/Intensity Refinement"};

    // "Half size of the search window for pattern corner features. 
    // There is a tradeoff here: on the one hand, it should be as small as possible 
    // to be able to detect features close to the image borders and include little distortion. 
    // On the other hand, it needs to be large enough to be able to detect features properly. 
    // Especially if corners are blurred, it is necessary to increase this extent.");
    int window_half_extent = 21;

    // Path to file containing predicted features, board setup, homography etc.
    std::string predictions_folder_fn = "";
    std::string image_folder_fn = "";
    std::string results_folder_fn = "";
    bool show_visualizations;

    app.add_option("-w,--window-half-extent", window_half_extent, "Window half extent (half size of search window)");
    app.add_option("-f,--predicted-feature-files", predictions_folder_fn, "YAML containing predictions");
    app.add_option("-i,--image_folder", image_folder_fn, "filename for image folders");
    app.add_option("-r,--results_folder", results_folder_fn, "filename for results folders");
    app.add_option("-s,--show_visualizations", show_visualizations, "option to show visualization");

    CLI11_PARSE(app, argc, argv);
    std::cout << "[CLI] window-half-extent read:" << window_half_extent << std::endl;
    std::cout << "[CLI] predicted feature fn read:" << predictions_folder_fn << std::endl;
    std::cout << "[CLI] results folder fn read:" << results_folder_fn << std::endl;
    std::cout << "[CLI] image folder fn read:" << image_folder_fn << std::endl;
    std::cout << "[CLI] show_visualizations read:" << show_visualizations << std::endl;

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
            image_folder_fn, // path to folder containing images.
            predictions_folder_fn,
            results_folder_fn,
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
