cmake_minimum_required(VERSION 3.10)
project(PointCloudProcessing)

set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

#PCL 1.8.1
find_package( PCL 1.8.1 REQUIRED )
add_definitions( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARIES_DIRS} )

add_executable(PointCloudProcessing main.cpp )
target_link_libraries(PointCloudProcessing
        smooth
        pcaAnalysis
        buildingFacadeExtraction
        ${PCL_LIBRARIES})