cmake_minimum_required(VERSION 3.10)

project(pcl_pipeline CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INC_DIR ${CMAKE_SOURCE_DIR}/include)

find_package(fmt CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)

set(SOURCES
    ${SRC_DIR}/helpers.cpp
)

add_executable(pcl_pmf_pipeline
    ${SRC_DIR}/pcl_pmf_pipeline.cpp
    ${SOURCES}
)

target_link_libraries(pcl_pmf_pipeline PRIVATE
    fmt::fmt-header-only
    ${PCL_LIBRARIES}
)

target_include_directories(pcl_pmf_pipeline PRIVATE
    ${INC_DIR}
)

add_executable(pcl_ransac_pipeline
    ${SRC_DIR}/pcl_ransac_pipeline.cpp
    ${SOURCES}
)

target_link_libraries(pcl_ransac_pipeline PRIVATE
    fmt::fmt-header-only
    ${PCL_LIBRARIES}
)

target_include_directories(pcl_ransac_pipeline PRIVATE
    ${INC_DIR}
)

add_executable(lidar_test
    ${SRC_DIR}/LidarSimpleScene.cpp
    ${SRC_DIR}/Ray.cpp
    ${SRC_DIR}/Plane.cpp
    ${SRC_DIR}/RectPlane.cpp
    ${SRC_DIR}/Box.cpp
)

target_link_libraries(lidar_test PRIVATE
    fmt::fmt-header-only
    ${PCL_LIBRARIES}
)

target_include_directories(lidar_test PRIVATE
    ${INC_DIR}
)
