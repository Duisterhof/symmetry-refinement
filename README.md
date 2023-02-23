# Symmetry Refinement (from Thomas Schöps' Geometric Calibration Toolbox)

Original paper: [Thomas Schöps, Viktor Larsson, Marc Pollefeys, Torsten Sattler, "Why Having 10,000 Parameters in Your Camera Model is Better Than Twelve", arXiv 2019.](https://arxiv.org/abs/1912.02908)
GitHub [repository](https://github.com/puzzlepaint/camera_calibration): "Accurate geometric camera calibration"

## Introduction
This repository ports only the symmetry refinement portion (intensity refinement portion WIP) of Thomas Schöps' work with geometric camera calibration. It also includes (optional) visualization tools used by Thomas Schöps within his repository.

## Building


The following external dependencies are required.

| Dependency   | Version(s) known to work |
| ------------ | ------------------------ |
| [Boost](https://www.boost.org/) | 1.54.0 |
| [CUDA](https://developer.nvidia.com/cuda-downloads) | 10.1 |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) | 3.3.7 |
| [GLEW](http://glew.sourceforge.net/build.html) | 1.10.0 |
| [OpenGV](https://github.com/laurentkneip/opengv) | Commit 306a54e6c6b94e2048f820cdf77ef5281d4b48ad |
| [Qt](https://www.qt.io/) | 5.12.0; minimum version: 5.8 |
| [SuiteSparse](http://faculty.cse.tamu.edu/davis/suitesparse.html) | 4.2.1 |
| [zlib](https://zlib.net/) | - |

The following external dependencies are optional.

| Dependency   | Purpose |
| ------------ | ------- | 
| [librealsense2](https://github.com/IntelRealSense/librealsense) | Live input from RealSense D400 series depth cameras (tested with the D435 only). |
| [Structure SDK](https://structure.io/developers) | Live input from Structure Core cameras (tested with the color version only). To use this, set the SCSDK_ROOT CMake variable to the SDK path. |

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CUDA_FLAGS="-arch=sm_61" ..
make -j camera_calibration  # Reduce the number of threads if running out of memory, e.g., -j3
```

## Using

### Set up predictions YAML for each corresponding image.

Predictions YAML need the following information:

| **YAML Parameter Name**   | **Description**                                                                                                              |
|---------------------------|------------------------------------------------------------------------------------------------------------------------------|
| num_star_segments         | Number of star segments in the pattern                                                                                       |
| features                  | List of features detected within the pattern. Order must correspond to list of star_points.                                  |
| star_points               | List of star points in pattern coordinates. Order must correspond to list of features.                                       |
| pattern_coordinates_scale | Scale of pattern coordinates (to scale pattern coordinates to be between [-0.5/0.5] when (0,0) corresponds to feature point. |

Refer to `predictions/sample.yaml` for a sample.

The filename of each prediction yaml in your predictions folder has to correspond to an image in your image folder in order for feature refinement to work.

I.e. if you have an image called `sample.png`, you require a .yaml in predictions called `sample.yaml` before feautre refinement can be run.

### Set necessary flags through command line after building.

| Parameter               | Flag                          | Description                                                                  |
|-------------------------|-------------------------------|------------------------------------------------------------------------------|
| window half extent      | -w, --window-half-extent      | Half size of search window for pattern corner features.                      |
| predicted feature files | -f, --predicted-feature-files | Path to folder containing predictions.                                       |
| image folders           | -i, --image_folder            | Path to folder containing images.                                            |
| results folder          | -r, --results_folder          | Path to folder containing results (to be written).                           |
| show visualizations     | -s, --show-visualizations     | Boolean (requires "true/false" after flag) that turns visualizations on/off. |


### Output

After proper setup, you should receive *.csv files within your results folder corresponding to image/prediction names.

csvs are of the following format

bool feature_found_by_symmetry , feature_point_x (px) , feature_point_y (px)


