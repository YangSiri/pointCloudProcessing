#PCL 1.8.1
find_package( PCL 1.8 REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
list (REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

LIST(APPEND CMAKE_MODULE_PATH "/home/cyz/CLionProjects/PointCloudProcessing/cmake/")
#boost
set(BOOST_ROOT "/opt/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

#eigen3
find_package(Eigen3 REQUIRED)
#add_definitions(${EIGEN_DEFINITIONS})
#include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

#opencv
find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
LINK_LIBRARIES( ${OpenCV_LIBRARY_DIRS} )

# Glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog libglog REQUIRED)

#gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
LINK_DIRECTORIES(${GTSAM_LIBRARY_DIRS})

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
     #Attempt to find package-based kindr
    pkg_check_modules(kindr kindr REQUIRED)
endif()

#sophus
find_package(Sophus REQUIRED)


include_directories(
        include
        include/curves
        ${kindr_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
)

#add libraries
add_library(polynomial_splines_traits SHARED hpp-cpp/polynomial_splines_traits.cpp)
target_link_libraries(polynomial_splines_traits ${Boost_LIBRARIES} glog ${OpenCV_LIBRARIES})

add_library(cubicHermiteSE3curve SHARED CubicHermiteSE3Curve.cpp
        hpp-cpp/helpers.cpp
        hpp-cpp/KeyGenerator.cpp
        hpp-cpp/SlerpSE3Curve.cpp
        hpp-cpp/SE3Curve.cpp)
target_link_libraries(cubicHermiteSE3curve ${Boost_LIBRARIES} glog )

add_library(smooth SHARED smoothing.cpp)
target_link_libraries(smooth ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_library(pcaAnalysis SHARED pcaAnalysis.cpp )
target_link_libraries(pcaAnalysis smooth ${PCL_LIBRARIES} )

add_library(buildingFacadeExtraction SHARED buildingFacadeExtraction.cpp planeFactorTest.cpp)
target_link_libraries(buildingFacadeExtraction pcaAnalysis smooth ${PCL_LIBRARIES} )

add_library(lidarOdometryLib SHARED lidarOdometry.cpp)
target_link_libraries(lidarOdometryLib smooth ${PCL_LIBRARIES})

add_library(pcapReader SHARED pcapReader.cpp)
target_link_libraries(pcapReader ${PCL_LIBRARIES} PacketDriver PacketDecoder)

add_library(ringProjection SHARED ringProjection.cpp)

#//////////////////////////////////////////////////////////////////////////
#add excutables
add_executable(lidarOdometry mainFunc.cpp)
target_link_libraries(lidarOdometry lidarOdometryLib pcapReader pcap)

add_executable(dynaObjTest dynaObjTest.cpp)
target_link_libraries(dynaObjTest
        ${OCTOMAP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        glog
        ${OpenCV_LIBRARIES}
        polynomial_splines_traits
        cubicHermiteSE3curve
        ringProjection
        )

add_executable(planeFactorTest planeFactorTest.cpp)
target_link_libraries(planeFactorTest
#        ${OCTOMAP_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
#        glog
#        ${OpenCV_LIBRARIES}
#        ${Sophus_LIBRARIES}
        gtsam
        smooth
        )

add_executable(relocalizationTest relocalizationTest.cpp)
target_link_libraries(relocalizationTest
        ${PCL_LIBRARIES}
        )

add_executable(trjectoryIntepolateTest trjectoryIntepolateTest.cpp)
target_link_libraries(trjectoryIntepolateTest
        smooth
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES}
        )