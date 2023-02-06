set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Find includes in corresponding build directories.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Instruct CMake to run rcc (resource compiler) automatically when needed.
set(CMAKE_AUTORCC ON)

add_subdirectory(third_party/sophus)

find_package(Qt5 5.8 REQUIRED COMPONENTS Core OpenGL Widgets)
# find_package(Qt5X11Extras REQUIRED)
find_package(Threads REQUIRED)

set(VISUALIZER
    third_party/puzzlepaint_visualizer/image_display.h
    third_party/puzzlepaint_visualizer/image_display.cc
    third_party/puzzlepaint_visualizer/image_display_qt_widget.h
    third_party/puzzlepaint_visualizer/image_display_qt_widget.cc    
    third_party/puzzlepaint_visualizer/image_display_qt_window.h
    third_party/puzzlepaint_visualizer/image_display_qt_window.cc
    third_party/puzzlepaint_visualizer/any_image.h
    third_party/puzzlepaint_visualizer/image.cc
    third_party/puzzlepaint_visualizer/qt_thread.cc
    third_party/puzzlepaint_visualizer/qt_thread.h
    third_party/puzzlepaint_visualizer/window_callbacks.h
    third_party/puzzlepaint_visualizer/libvis.cc
    third_party/puzzlepaint_visualizer/libvis.h
    third_party/puzzlepaint_visualizer/image_io_libpng.cc
    third_party/puzzlepaint_visualizer/image_io_libpng.h
    third_party/puzzlepaint_visualizer/image_io_netpbm.cc
    third_party/puzzlepaint_visualizer/image_io_netpbm.h
    third_party/puzzlepaint_visualizer/image_io_qt.cc
    third_party/puzzlepaint_visualizer/image_io_qt.h
    third_party/puzzlepaint_visualizer/image_io.cc
    third_party/puzzlepaint_visualizer/image_io.h
    third_party/puzzlepaint_visualizer/eigen.h
    third_party/puzzlepaint_visualizer/image.h
    third_party/puzzlepaint_visualizer/image_template_specializations.h
    third_party/puzzlepaint_visualizer/calibration_window.cc
    third_party/puzzlepaint_visualizer/calibration_window.h
    third_party/puzzlepaint_visualizer/dataset.cc
    third_party/puzzlepaint_visualizer/dataset.h
    third_party/puzzlepaint_visualizer/sophus.h
    ${GENERATED_HEADERS}
    third_party/resources/resources.qrc
)

add_library(symmetry_refinement_visualizer SHARED ${VISUALIZER})
target_include_directories(symmetry_refinement_visualizer
    PUBLIC
    ./libvis/src
    third_party/puzzlepaint_visualizer
    third_party/sophus
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Opengl_INCLUDE_DIRS}
)
target_link_libraries(symmetry_refinement_visualizer 
    PUBLIC
    ${Qt5Opengl_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5X11Extras_LIBRARIES}
)
set_target_properties(symmetry_refinement_visualizer PROPERTIES LINKER_LANGUAGE CXX)

if(Qt5Widgets_FOUND)
    target_compile_options(symmetry_refinement_visualizer PUBLIC -DLIBVIS_HAVE_QT)
endif()

# Building symmetry_exec
set(SYMMETRY_REFINEMENT_INCLUDES

)

set(SYMMETRY_REFINEMENT_EXECUTABLES
    include/cpu_refinement_by_symmetry.hpp
    include/feature_detector.hpp
    src/feature_detector.cpp
    src/cpu_refinement.cpp
    src/main.cpp
)

add_executable(symmetry_exec 
    ${SYMMETRY_REFINEMENT_EXECUTABLES})

target_include_directories(symmetry_exec
    PRIVATE 
    third_party/sophus
    src
    include
    third_party
)

target_link_libraries(symmetry_exec
    PRIVATE
    Threads::Threads
    symmetry_refinement_visualizer
)
