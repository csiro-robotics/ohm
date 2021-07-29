# Occupancy Homogeneous Map

The ohm library is a probabilistic voxel occupancy map supporting fast GPU based population and operations and normal distribution transform semantics. The ohm library defines an occupancy map consisting of regions or chunks of homogeneous voxels, arranged in contiguous memory blocks. This homogeneous voxel layout, rather than an octree layout, supports fast GPU based map population using OpenCL and CUDA.

## Building

### Prerequisites and Requirements

The ohm library supports both OpenCL and CUDA GPU processing. OpenCL development focuses on GCC and Visual Studio running on an Intel OpenCL GPU device. AMD and NVIDIA GPUs have been tested and should also function. The CUDA implementation has been built for compute 5.0 and 6.0. Other architectures may work, but must be specifically tested.

Building ohm requires:

- C++14 compatible compiler such as:
  - GCC 5.7
  - Visual Studio 15 2017
  - APPLE LLVM 9.1
- For OpenCL
  - An OpenCL 1.2 or 2.0 SDK. 1.2 must be used if running on NVIDIA hardware.
  - OpenCL 1.2 runtime. OpenCL 2.x also supported (command line selectable)
- For CUDA
  - CUDA 10

The following 3rd-party libraries are required to build ohm:

- Either an [OpenCL SDK](./OpenCL.md) or the [CUDA](https://developer.nvidia.com/cuda-zone) SDK
- [ZLib](https://www.zlib.net/) for serialisation compression.
- [CMake](https://cmake.org/) for project set up
- [OpenGL Mathematics (GLM)](https://glm.g-truc.net/) for 3D data types.

[Googletest](https://github.com/google/googletest) is also used when building unit tests, however, this is downloaded as part of the build.

Additional, the following 3rd-party libraries may optionally be used:

| Library                                                                     | Feature Usage                                                                           |
| --------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [3rd Eye Scene](https://github.com/data61/3rdEyeScene)                      | For debug visualisation of map generation.                                              |
| [Doxygen](http://www.doxygen.nl/)                                           | For generation of API documentation.                                                    |
| [Eigen3](http://eigen.tuxfamily.org/index.php)                              | Used in small amounts in some tests and as a faster option for some geometry operations |
| [Intel Threading Building Blocks](https://www.threadingbuildingblocks.org/) | Multi-threaded CPU operations.                                                          |
| [GLEW](http://glew.sourceforge.net/)                                        | For HeightmapImage in ohmheightmaputil                                                  |
| [GLFW](https://www.glfw.org/)                                               | For HeightmapImage in ohmheightmaputil                                                  |
| [libpng](http://www.libpng.org/)                                            | To convert heightmap to image using utils/ohmhm2img                                     |
| [PDAL](https://pdal.io/)                                                    | Load point various point cloud formats for ohmpop.                                      |

While efforts are made to ensure components remain optional, certain configurations may be incompatible.

#### Ubuntu apt Packages

On Ubuntu, the required packages may be installed using the following command:

```bash
sudo apt install cmake zlib1g-dev libglm-dev googletest
```

Setup of OpenCL requires mode detailed [instructions (link)](./OpenCL.md).

For CUDA setup instructions, visit [NVIDIA CUDA Zone](https://developer.nvidia.com/cuda-zone).

Additional, recommended packages can be installed using:

```bash
sudo apt install libtbb-dev libpdal-dev doxygen
```

Optional packages for heightmap generation and image conversion:

```bash
sudo apt install libglew-dev libglfw3-dev libpng-dev libeigen3-dev
```

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
- Using the OpenCL 2.x SDK and selecting an NVIDIA GPU will result in runtime crashes.
- When installing, OHM_EMBED_GPU_CODE must be defined in order to run ohmocl; otherwise OpenCL source is not found.

## Resolving OpenCL SDK With Multiple Options

There are some pitfalls in trying to resolve an appropriate OpenCL SDK when multiple SDK options are installed. Most notably selecting between the Intel SDK over the NVIDIA SDK can be problematic. The best way to ensure the correct option is selected is to ensure the desired SDK prefix path is set in the PATH environment variable as the before other SDK paths. The prefix path, is essentially the path containing the `include` and `lib` directories.

While this can be changed by explicitly changing `OpenCL_INCLUDE_DIR` and `OpenCL_LIBRARY` in the CMake cache (using `ccmake` or `cmake-gui`) this can still have issues with selecting a particular API version. This is because the available API versions are determined when the OpenCL header is first found and will not be updated if the paths are explicitly changed. To combat this, edit the `CMakeCache.txt` file and remove all `OPENCL_VERSION_X_X` entries.
