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
include_directories( src include )
include_directories( ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        #        ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIR}
        )

include_directories( ${CHOLMOD_INCLUDE_DIR} )
include_directories( ${G2O_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(odom
        include/util.cpp
        include/beam_separate.cpp
        include/beam_separate.h
        include/util.h
        include/lego_feature.cpp
        include/lego_feature.h
        include/a_loam_odom.cpp
        include/a_loam_odom.h
        include/FP_PC_crub.cpp
        include/FP_PC_crub.h
        include/FP_PC_viewing.h
        include/FP_PC_viewing.cpp
        include/mapping.cpp
        include/mapping.h)
add_executable(mapping pcd_reader.cpp include/main.h)
target_link_libraries(mapping ${PCL_LIBRARIES}
        odom
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${CSPARSE_LIBRARY}
        g2o_csparse_extension
        g2o_types_slam3d)
