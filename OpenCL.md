# OpenCL Usage

OpenCL is one of the two GPU library options required for the Occupancy map (ohm). Most development has occurred on top of the Intel OpenCL implementation, but the NVIDIA implementation also works. These instructions focus on installation requirements for Intel OpenCL. For the NVIDIA support, download and install the [CUDA development kit](https://developer.nvidia.com/cuda-zone).

Both runtime drivers and an SDK need to be installed in order to get it running. Getting Intel drivers running can be an issue so the following instructions are maintained to help the installation process. The following process is focused entirely on Ubuntu 18.04.

## Windows SDK and Drvier Installation

The [Intel OpenCL SDK](https://software.intel.com/en-us/intel-opencl) is recommended for Windows development. Follow the installation instructions in the link.

Note that there are some pathing issues which may cause issues under Windows, especially when the CUDA SDK is also installed. It is recommended that the PATH environment variable is set up to find the Intel OpenCL DLLs and executables before the NVIDIA implementation is located.

OpenCL runtime drivers are installed as part of the video card driver package.

## Linux SDK Installation

The Intel OpenCL SDK is also available for Linux, however, for most Debian installations it is enough to install the appropriate apt packages provided the correct device drivers are installed.

```
sudo apt-get install opencl-headers ocl-icd-dev ocl-icd-libopencl1 ocl-icd-opencl-dev
```

### Installing Intel OpenCL Drivers

Extensive Intel CPU/GPU testing has not been made, but the following information may help address some OpenCL standard compatibility issues. The OpenCL standard defines the GPU code compilation, not the SDK version. This can be set on OHM_OPENCL_STD.

| Intel CPU Generation | Recommended OpenCL Standard |
| -------------------- | --------------------------- |
| < 6th Generation     | Unknown                     |
| 6th Generation       | 1.2                         |
| 7th Generation       | 2.0                         |
| 8th Generation       | 2.0                         |
| 9th Generation       | 2.0                         |

There are three different drivers which may be relevant to the installation process.

1. Intel Linux drivers:
    - Direct link: https://github.com/intel/compute-runtime/releases
    - General information https://software.intel.com/en-us/articles/opencl-drivers
2. beignet: apt package for Intel NEO drivers.
3. Intel Legacy Linux drivers: https://software.intel.com/en-us/articles/legacy-opencl-drivers#latest_linux_SDK_release

For general installation:

- Determine your CPU generation:
    - `grep -m 1 name /proc/cpuinfo`
    - Results should look something like: `Intel(R) Core(TM) i7-6700 CPU @ 3.40GHz` The first digit of the CPU number - in this case 6700 - indicates the generation '6'.
- Install the most appropriate driver (see below) based on your CPU generation.
    - For 6th generation use the legacy drivers (option 3).
    - For 7th+ generation use the current Intel Drives (option 1).
- Install `clinfo` apt package to confirm successful installation
    - `sudo apt-get install clinfo`
- Verify installation:
    - `clinfo | grep "Device Name"`

If installation is successful you should see a results like the following:

```
  Device Name                                     GeForce GT 640
  Device Name                                     Intel(R) HD Graphics
  Device Name                                     Intel(R) Core(TM) i7-6700 CPU @ 3.40GHz
```

Not that this example is also listing a CUDA device.

#### Intel Linux Driver

This installs the current Intel OpenCL drivers for 7th, 8th and 9th generation CPU. These are the perferred drivers as they are continually updated by Intel. Drivers may be downloaded from https://github.com/intel/compute-runtime/releases with accompanying installation instructions.

These drivers are incompatible with beignet.

#### Beignet

In 18.04 these are drivers for the NEO architecture used in 7th, 8th and 9th generation core processors. Run the following command to install the Beignet drivers:

```
sudo apt-get install beignet-dev
```

Verify the installation (above).

If this fails, you will need to `apt-get purge <package>` the following apt-packages before continuing:

- beignet
- beignet-dev
- beignet-opencl-icd

#### Intel Legacy Linux Driver

Legacy drivers are appropriate only for 6th (and 5th) generation intel chips. These are downloaded from https://software.intel.com/en-us/articles/legacy-opencl-drivers#latest_linux_SDK_release and the installation process is described at http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_intel-opencl-installation.pdf

These drivers are incompatible with beignet.

## NVidia OpenCL Compatibility

NVidia OpenCL requires installation of official NVidia drivers for your video card. Discussion of how to do so is beyond the scope of this document. Use `clinfo | grep "Device Name"` to determine whether NVidia drivers are correctly installed. You should see an NVidia related device name such as "GeForce XXX".

In order to build ohm to run on NVidia, the library must be build for OpenCL 1.2 (host API) and OpenCL 1.2 runtime (device standard). To do so, configure the following CMake variables to "1.2" using `ccmake`, `cmake-gui`, or editing the `CMakeCache.txt` file directly.

- `OHM_OPENCL_STD` : 1.2 (OpenCL runtime compilation)
- `OHM_OPENCL_SDK_VER` : 1.2 (OpenCL SDK selection)

Note that NVidia performance will generally be worse than Intel OpenCL because of the way ohm uses global memory.
