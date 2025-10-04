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
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/helpers.cpp
)

add_executable(pcl_main
    ${SOURCES}
)

target_link_libraries(pcl_main PRIVATE
    fmt::fmt-header-only
    ${PCL_LIBRARIES}
)

target_include_directories(pcl_main PRIVATE
    ${INC_DIR}
)
