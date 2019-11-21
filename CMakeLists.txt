cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_compare)
find_package(PCL REQUIRED)
set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )
set( CMAKE_EXPORT_COMPILE_COMMANDS on)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( yaml-cpp REQUIRED )
#找到各种sb库
include_directories(${YAML_CPP_SOURCE_DIR}/src)
include_directories(${YAML_CPP_SOURCE_DIR}/test)
#测试point to plane icp
find_package(libpointmatcher CONFIG)
FIND_PACKAGE( CSparse )
find_package( G2O REQUIRED )
find_package( Eigen3  REQUIRED)
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

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

#1.sophus
ADD_LIBRARY(sophus SHARED include/sophus/se3.cpp
        include/sophus/so3.cpp
        include/sophus/so3.h
        include/sophus/se3.h)
#2.mapping lm
add_library(mapping_lm
        include/mapping/lmOptimizationSufraceCorner.cpp
        include/mapping/lmOptimizationSufraceCorner.h
        )
target_link_libraries(mapping_lm sophus)
#3. odometry
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
        include/FirstPersonViewFilter/FP_PC_viewing.cpp)
#        include/mapping/mapping.cpp
#        include/mapping/mapping.h)
target_link_libraries(odom ${OpenCV_LIBS})


#4. ground segmentation 地面分割
add_library(ground_seg
        include/oneFrameGND/ground_seg.h
        include/oneFrameGND/ground_seg.cpp
        )
target_link_libraries(ground_seg        yaml-cpp ${PCL_LIBRARIES})

#5. spline fusion 插值
add_library(spline
        include/spline/experiment.cpp
        include/spline/experiment.h
        )
target_link_libraries(spline ${PCL_LIBRARIES} sophus)

#6. pose graph IO

ADD_LIBRARY(poseIO SHARED include/g2oIO/PoseGraphIO.h include/g2oIO/PoseGraphIO.cpp)
target_link_libraries(poseIO      g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})
#main program
add_executable(mapping main.cpp include/otherFunctions.h)
target_link_libraries(mapping ${PCL_LIBRARIES}
        odom
        ground_seg
        ${OpenCV_LIBS}
        mapping_lm
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${CSPARSE_LIBRARY}
        g2o_csparse_extension
        g2o_types_slam3d
        poseIO
        spline)
