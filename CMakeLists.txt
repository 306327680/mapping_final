cmake_minimum_required(VERSION 3.6.3 FATAL_ERROR)
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
#找到ros库
find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(sensor_msgs REQUIRED)
include_directories(${sensor_msgs_INCLUDE_DIRS})

find_package(rosbag REQUIRED)
include_directories(${rosbag_INCLUDE_DIRS})

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})

find_package(nav_msgs REQUIRED)
include_directories(${nav_msgs_INCLUDE_DIRS})

find_package(geometry_msgs REQUIRED)
include_directories(${geometry_msgs_INCLUDE_DIRS})
find_package(tf_conversions REQUIRED)
include_directories(${tf_conversions_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})
#libpointmatcher
find_package(libpointmatcher CONFIG)
#QT

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

set(CMAKE_AUTOMOC ON) # 设置自动生成moc文件,AUTOMOC打开可以省去QT5_WRAP_CPP命令
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # 设置自动生成ui.h文件,AUTOUIC打开可以省去QT5_WRAP_UI命令


include_directories( src include ${libpointmatcher_INCLUDE_DIRS})

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
#[[add_definitions(${PCL_DEFINITIONS})]]
#g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
#ceres
find_package( Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
#open mp
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
        include/DataIO/ReadBag.cpp include/DataIO/ReadBag.h
        include/registration/registration.cpp
        include/registration/registration.h
        include/GPS/gpsTools.cpp include/GPS/gpsTools.h
)
target_link_libraries(ground_seg
        ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES}
        ${rosbag_LIBRARIES} ${nav_msgs_LIBRARIES}
        ${geometry_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES}
        yaml-cpp ${PCL_LIBRARIES}
        ${libpointmatcher_LIBRARIES}
        ${cv_bridge_LIBRARIES})

#5. spline fusion 插值
add_library(spline
        include/spline/experiment.cpp
        include/spline/experiment.h
        )
target_link_libraries(spline ${PCL_LIBRARIES} sophus)

#6. pose graph IO

ADD_LIBRARY(poseIO SHARED include/g2oIO/PoseGraphIO.h include/g2oIO/PoseGraphIO.cpp)
target_link_libraries(poseIO g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})
#7. NDT omp
add_library(ndt_omp
        include/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        include/ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        include/ndt_omp/src/pclomp/ndt_omp.cpp
        include/ndt_omp/src/pclomp/gicp_omp.cpp
        )
target_link_libraries(ndt_omp      ${PCL_LIBRARIES})

##qt
qt5_wrap_ui(ui_FILES include/mainwindow.ui)
#[[qt5_add_resources(qrc_FILES drawable.qrc)]]



#main program
add_executable(mapping main.cpp
        include/otherFunctions.h
        include/6DOFcalib/Calibration6DOF.cpp
        include/6DOFcalib/Calibration6DOF.h
        ${ui_FILES}
        include/qt/mainwindow.cpp
        include/qt/mainwindow.h
        include/imgAddColor2Lidar/imgAddColor2Lidar.cpp
        include/imgAddColor2Lidar/imgAddColor2Lidar.h
        include/CSV/CSVio.cpp include/CSV/CSVio.h include/otherFunctions.cpp include/VO/VisualOdom.cpp include/VO/VisualOdom.h include/loopClosure/loopClosure.cpp include/loopClosure/loopClosure.h include/GPS_constraint_mapping/GPS_loop_mapping.cpp include/GPS_constraint_mapping/GPS_loop_mapping.h include/DynamicRemoveMap/DynamicRemoveMap.cpp include/DynamicRemoveMap/DynamicRemoveMap.h)

target_link_libraries(mapping ${PCL_LIBRARIES}
        ${tf_conversions_LIBRARIES}
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
        spline
        ndt_omp
        ${CERES_LIBRARIES}
        Qt5::Widgets
        Qt5::Core
        Qt5::Gui)
