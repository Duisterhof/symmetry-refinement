#include <iostream>
#include <filesystem> // directory_iterator
#include <memory> // unique_ptr
#include <thread> // thread
#include "cpu_refinement_by_symmetry.hpp"

#include "puzzlepaint_visualizer/libvis.h"
#include "puzzlepaint_visualizer/dataset.h"
#include "puzzlepaint_visualizer/image_display.h"
#include "puzzlepaint_visualizer/calibration_window.h"
#include <unistd.h>
// void ExtractFeatures(std::string image_directory) {
    // std::filesystem::directory_iterator it(image_directory), end;
// }

using namespace vis;

void refine_features(std::string path_to_image, std::string image_folder, CalibrationWindow* calibration_window) {
    Dataset dataset(1); // creating a single dataset.

    std::filesystem::path fp(path_to_image);
    std::filesystem::path image_fn(image_folder);
    
    // Iterating through images within specified folder.
    for (auto const& dir_entry : std::filesystem::directory_iterator{fp/image_fn}) 
    {
        vis::Image<vis::Vec3u8> image;
        vis::Image<vis::u8> gray_image;
        image.ConvertToGrayscale(&gray_image);

    }     
}

int main(int argc, char** argv)
{
    bool show_visualizations = true;
    QApplication qapp(argc, argv);
    qapp.setQuitOnLastWindowClosed(false);

    // Create the main window.
    CalibrationWindow calibration_window(nullptr, Qt::WindowFlags());
    calibration_window.show();
    calibration_window.raise();

    // Starting a thread for UI to work while feature refinement is underway.
    std::thread calibrate_thread([&]{
        refine_features(std::string("/home/siheng/dha/symmetry"), std::string("image"));

        RunInQtThreadBlocking([&]() {
        if (calibration_window.isVisible()) {
            qapp.setQuitOnLastWindowClosed(true);
        } else {
            qapp.quit();
        }
        });
    });

    // Run the Qt event loop
    qapp.exec();

    calibrate_thread.join();
    return EXIT_SUCCESS;

    return 0;
}
