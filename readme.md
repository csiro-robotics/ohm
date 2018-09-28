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
- An OpenCL SDK.

Additional, the following 3rd-party libraries may optionally be used:

- [Google Test](https://github.com/google/googletest) for unit tests.
- [libLAS](https://liblas.org/) to generate maps from LAS file point clouds.
- [LASZip](https://laszip.org/) to optionally support compressed LAS files (LAZ files)
- [Intel Threading Building Blocks](https://www.threadingbuildingblocks.org/) to support multi-threaded CPU operations.
- [Boost](https://www.boost.org/) required by libLAS
- [3rd Eye Scene](https://github.com/data61/3rdEyeScene) for debug visualisation of map generation.
- [Doxygen](http://www.doxygen.nl/) for generation of API documentation.

While efforts are made to ensure certain components remain optional, certain configurations may be incompatible.

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
