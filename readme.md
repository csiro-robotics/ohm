# Occupancy Homogeneous Map

The ohm library is a probabilistic voxel occupancy map supporting fast GPU based population and operations. The ohm library defines an occupancy map consisting of regions or chunks of homogeneous voxels, arranged in continguous memory blocks. This homogeneous voxel layout, rather than an octree layout, supports fast GPU based map population using Intel OpenCL GPUs.

## Building

### Prerequisites and Requirements

Development of the ohm library focuses on GCC and Visual Studio running on an Intel OpenCL GPU device. AMD and NVIDIA GPUs are not guaranteed to work and compatibility with Intel GPUs may vary. ohm requires:

- C++14 compatible compiler such as:
  - GCC 5.7
  - Visual Studio 15 2017
  - APPLE LLVM 9.1
- OpenCL 1.2

The following 3rd-party libraries are required to build ohm:

- [ZLib](https://www.zlib.net/) for serialisation compression.
- [CMake](https://cmake.org/) for project set up
- [OpenGL Mathematics (GLM)](https://glm.g-truc.net/) for 3D data types.
- An [OpenCL SDK](./OpenCL.md).

Additional, the following 3rd-party libraries may optionally be used:

- [Google Test](https://github.com/google/googletest) for unit tests.
- To generate maps from LAS file point clouds, either:
    - [PDAL](https://pdal.io/) to load point clouds for gene
    - [libLAS](https://liblas.org/) and [LASZip](https://laszip.org/)
        - [Boost](https://www.boost.org/) required by libLAS
- [Intel Threading Building Blocks](https://www.threadingbuildingblocks.org/) to support multi-threaded CPU operations.
- [3rd Eye Scene](https://github.com/data61/3rdEyeScene) for debug visualisation of map generation.
- [Doxygen](http://www.doxygen.nl/) for generation of API documentation.

While efforts are made to ensure certain components remain optional, certain configurations may be incompatible.

#### Ubuntu apt Packages
On Ubuntu, the required packages may be installed using the following command:
```
sudo apt-get install cmake zlib1g-dev libglm-dev libpdal-dev libtbb-dev doxygen googletest
```

Google test must also be built as the package is source only:
```
cd /usr/src/gtest/build-aux
sudo cmake ..
sudo make
sudo cp *.a /usr/lib
```

Setup of OpenCL requires mode detailed [instructions (link)](./OpenCL.md).

Additional experimental (TODO: make this comment good):

- libglew-dev
- libglfw3-dev
- libpng-dev

### Build Instructions

1. Download source code.
2. Install pre-requisites.
3. Configure the project.
    - Create a `build` subdirectory and change into this subdirectory.
    - Execute `cmake ..`
        - For Visual Studio configuration, include the option `-G Visual Studio 15 2017 Win64`
    - Build the project
        - For Visual Studio, open the solution file and build.
        - For make based platforms, run `make -j`

## Notable Known Issues

- OpenCL compatibility with certain devices may vary.
- OpenCL performance on various devices may vary especially with memory transfer rates.
- There is mention of CUDA in the code, but there is currently no CUDA compatibilty.
- Installation scripts have yet to be verified.
- A coding standards pass will be shortly effected which will significantly change file and variable naming.

## Resolving OpenCL SDK With Multiple Options

There are some pitfalls in trying to resolve an appropriate OpenCL SDK when multiple SDK options are installed. Most notably selecting between the Intel SDK over the NVIDIA SDK can be problematic. The best way to ensure the correct option is selected is to ensure the desired SDK prefix path is set in the PATH environment variable as the before other SDK paths. The prefix path, is essentially the path containing the `include` and `lib` directories.

While this can be changed by explicitly changing `OpenCL_INCLUDE_DIR` and `OpenCL_LIBRARY` in the CMake cache (using `ccmake` or `cmake-gui`) this can still have issues with selecting a particular API version. This is because the available API versions are determined when the OpenCL header is first found and will not be updated if the paths are explicitly changed. To combat this, edit the `CMakeCache.txt` file and remove all `OPENCL_VERSION_X_X` entries.
