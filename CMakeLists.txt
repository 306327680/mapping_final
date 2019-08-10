cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_compare)
find_package(PCL REQUIRED)
set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )
set( CMAKE_EXPORT_COMPILE_COMMANDS on)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
FIND_PACKAGE( CSparse )
find_package( G2O REQUIRED )
find_package( Eigen3 REQUIRED)
find_package( Cholmod REQUIRED )
find_package( Sophus REQUIRED )
find_package(OpenCV REQUIRED)
include_directories( src include )
include_directories( ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        include
        )
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories( ${CHOLMOD_INCLUDE_DIR} )
include_directories( ${G2O_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})
ADD_LIBRARY(sophus SHARED include/sophus/se3.cpp include/sophus/so3.cpp)

add_library(mapping_lm
        include/mapping/lmOptimizationSufraceCorner.cpp
        include/mapping/lmOptimizationSufraceCorner.h
        )


add_library(odom
        include/tools/util.cpp
        include/distortion/beam_separate.cpp
        include/distortion/beam_separate.h
        include/tools/util.h
        include/FeatureExtraction/lego_feature.cpp
        include/FeatureExtraction/lego_feature.h
        include/odometry/a_loam_odom.cpp
        include/odometry/a_loam_odom.h
        include/FirstPersonViewFilter/FP_PC_crub.cpp
        include/FirstPersonViewFilter/FP_PC_crub.h
        include/FirstPersonViewFilter/FP_PC_viewing.h
        include/FirstPersonViewFilter/FP_PC_viewing.cpp
        include/mapping/mapping.cpp
        include/mapping/mapping.h)
add_executable(mapping pcd_reader.cpp include/otherFunctions.h)
target_link_libraries(mapping ${PCL_LIBRARIES}
        odom
        mapping_lm
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${CSPARSE_LIBRARY}
        g2o_csparse_extension
        g2o_types_slam3d)
