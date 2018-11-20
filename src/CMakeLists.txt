#PCL 1.8.1
find_package( PCL 1.8.1 REQUIRED )
add_definitions( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARIES_DIRS} )

LIST(APPEND CMAKE_MODULE_PATH "/home/cyz/CLionProjects/PointCloudProcessing/cmake/")
#boost
set(BOOST_ROOT "/opt/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
#eigen3
find_package(Eigen3 REQUIRED)
#add_definitions(${EIGEN_DEFINITIONS})
include_directories(${EIGEN_INCLUDE_DIR})

add_library(smooth smoothing.cpp)
target_link_libraries(smooth ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_library(pcaAnalysis  pcaAnalysis.cpp )
target_link_libraries(pcaAnalysis smooth ${PCL_LIBRARIES} )

add_library(buildingFacadeExtraction  buildingFacadeExtraction.cpp )
target_link_libraries(buildingFacadeExtraction pcaAnalysis smooth ${PCL_LIBRARIES} )

add_library(lidarOdometryLib lidarOdometry.cpp)
target_link_libraries(lidarOdometryLib smooth ${PCL_LIBRARIES})

add_library(pcapReader pcapReader.cpp)
target_link_libraries(pcapReader ${PCL_LIBRARIES})


add_executable(lidarOdometry mainFunc.cpp)
target_link_libraries(lidarOdometry lidarOdometryLib pcapReader)