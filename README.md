# PCL Examples

This repository features some examples using the C++ [Point Cloud Library (PCL)](https://pointclouds.org/).

## Building

This process assumes that you are using vcpkg as a dependency manager for [fmt](https://vcpkg.io/en/package/fmt.html) and [PCL](https://vcpkg.io/en/package/pcl).

You can get vcpkg [here](https://vcpkg.io) and take a look at how to get started [here](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started).

If you do not want to use vcpkg and build with another method, take a look [here for pcl](https://pointclouds.org/downloads/) and [here for fmt](https://fmt.dev/12.0/get-started/).


1. Clone this git repo into a local directory on your machine
    - `git clone https://github.com/squee72564/PCL_Examples.git`
2. CD into this cloned repo
    - `cd ./PCL_Examples`
3. Create a build directory
    - `mkdir ./build`
4. Get -DCMAKE_TOOLCHAIN_FILE to use with cmake
    - `vcpkg integrate install`
5. Run the cmake configuration step passing in your vcpkg toolchain file
    - `cmake -S . -B ./build -DCMAKE_TOOLCHAIN_FILE=/home/user/.local/share/vcpkg/scripts/buildsystems/vcpkg.cmake`
    - Note that the toolchain file will differ from the one above on your system
    - This can also take a very long time, as PCL has a ton of dependencies
6. Run the cmake build step
    - `cmake --build ./build`
7. You can now find the exectuables in the `./build` directory
