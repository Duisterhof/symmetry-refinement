set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find includes in corresponding build directories.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Instruct CMake to run rcc (resource compiler) automatically when needed.
set(CMAKE_AUTORCC ON)

find_package(Qt5 5.8 REQUIRED COMPONENTS Core OpenGL Widgets)
find_package(Qt5X11Extras REQUIRED)
find_package(Threads REQUIRED)
# if (TARGET Qt5::Core AND UNIX)
# # HACK, see: https://gitlab.kitware.com/cmake/cmake/issues/16915
# get_property( core_options TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS )
# string( REPLACE "-fPIC" "" new_core_options "${core_options}" )
# set_property( TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_core_options} )
# set_property( TARGET Qt5::Core PROPERTY INTERFACE_POSITION_INDEPENDENT_CODE "ON" )
# set( CMAKE_CXX_COMPILE_OPTIONS_PIE "-fPIC" )
# endif()
set(SYMMETRY_REFINEMENT
    include/cpu_refinement_by_symmetry.hpp
    src/cpu_refinement.cpp
)

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
    PRIVATE
    third_party/puzzlepaint_visualizer
    third_party/sophus
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Opengl_INCLUDE_DIRS}
)
target_link_libraries(symmetry_refinement_visualizer PUBLIC
    ${Qt5Opengl_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${OPENGL_LIBRARY}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5X11Extras_LIBRARIES}
)
set_target_properties(symmetry_refinement_visualizer PROPERTIES LINKER_LANGUAGE CXX)

if(Qt5Widgets_FOUND)
    message("HELLO")
    target_compile_options(symmetry_refinement_visualizer PUBLIC -DLIBVIS_HAVE_QT)
endif()

add_executable(symmetry_exec src/main.cpp)
target_include_directories(symmetry_exec PRIVATE src include third_party)
target_link_libraries(symmetry_exec
    PRIVATE
    Threads::Threads
    symmetry_refinement_visualizer
)
