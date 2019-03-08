# OpenCL Usage

OpenCL is required for the Occupancy map (ohm), however, it is specifically only tested with Intel OpenCL. NVidia OpenCL has been known to work, but is not as extensively tested.

Both runtime drivers and an SDK need to be installed in order to get it running. Getting Intel drivers running can be an issue so the following instructions are maintained to help the installation process. The following process is focused entirely on Ubuntu 18.04.

## Installing the OpenCL SDK

Intel publish an OpenCL SDK which includes kernel debugging capabilities. This is has not been tested here under Ubuntu and the Windows version tends to be somewhat difficult to get working. Thus, that avenue is only recommended if debugging is essential.

For most installations it is enough to install the appropriate apt packages.

```
sudo apt-get install opencl-headers ocl-icd-dev ocl-icd-libopencl1 ocl-icd-opencl-dev
```

## Intel CPU Compatibility

Extensive Intel CPU/GPU testing has not been made, but the following information may help address some OpenCL standard compatibility issues. The OpenCL standard defines the GPU code compilation, not the SDK version. This can be set on OHM_OPENCL_STD.

Intel CPU Generation    | Recommended OpenCL Standard
----------------------- | ---------------------------
< 6th Generation        | Unknown
6th Generation          | 1.2
7th Generation          | 2.0
8th Generation          | 2.0
9th Generation          | 2.0

### Installing Intel OpenCL Drivers

There are three different drivers which may be relevant to the installation process.

- beignet: apt package for Intel NEO drivers.
- Intel Linux drivers: https://software.intel.com/en-us/articles/opencl-drivers
- Intel Legacy Linux drivers: https://software.intel.com/en-us/articles/legacy-opencl-drivers#latest_linux_SDK_release


For general installation:

- Determine your CPU generation:
    - `grep -m 1 name /proc/cpuinfo`
    - Results should look something like: `Intel(R) Core(TM) i7-6700 CPU @ 3.40GHz` The first digit of the CPU number - in this case 6700 - indicates the generation '6'.
- Install `clinfo` apt package to confirm successful installation
    - `sudo apt-get install clinfo`
- Install the most appropriate driver (see below) based on your CPU generation.
    - For 6th generation use the legacy drivers.
    - For 7th+ generation, try beignet and the current Intel Drives.
- Verify installation:
    - `clinfo | grep "Device Name"`

If installation is successful you should see a results like the following:

```
  Device Name                                     GeForce GT 640
  Device Name                                     Intel(R) HD Graphics
  Device Name                                     Intel(R) Core(TM) i7-6700 CPU @ 3.40GHz
```

Not that this is also listing the CUDA device.

#### Beignet
First preference is to use the beignet package. In 18.04 these are drivers for the NEO architecture used in 7th, 8th and 9th generation core processors. This is the preferred approach, but may not work.

```
sudo apt-get install beignet-dev
```

Verify the installation (above).

If this fails, you will need to `apt-get purge <package>` the following apt-packages before continuing:

- beignet
- beignet-dev
- beignet-opencl-icd

#### Intel Linux Driver
This installs the current Intel OpenCL drivers for 7th, 8th and 9th generation CPU. The beignet drivers are preferred since Unbutu is not officially supported. The downloaded instructions are available hereat https://software.intel.com/en-us/articles/opencl-drivers.

- Download the .deb runtime driver from https://github.com/intel/compute-runtime/releases
- Run: `sudo dpkg -i <pkg>.deb`
- Verify installation (above)

These drivers are incompatible with beignet.

#### Intel Legacy Linux Driver
Legacy drivers are appropriate only for 6th (and 5th) generation intel chips. These are downloaded from https://software.intel.com/en-us/articles/legacy-opencl-drivers#latest_linux_SDK_release and the installation process is described at http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_intel-opencl-installation.pdf

These drivers are incompatible with beignet.

## NVidia OpenCL Compatibility

NVidia OpenCL requires installation of official NVidia drivers for your video card. Discussion of how to do so is beyond the scope of this document. Use `clinfo | grep "Device Name"` to determine whether NVidia drivers are correctly installed. You should see an NVidia related device name such as "GeForce XXX".

In order to build ohm to run on NVidia, the library must be build for OpenCL 1.2 (host API) and OpenCL 1.2 runtime (device standard). To do so, configure the following CMake variables to "1.2" using `ccmake`, `cmake-gui`, or editing the `CMakeCache.txt` file directly.

- `OHM_OPENCL_STD` : 1.2 (OpenCL runtime compilation)
- `OHM_OPENCL_SDK_VER` : 1.2 (OpenCL SDK selection)

Note that NVidia performance will generally be worse than Intel OpenCL because of the way ohm uses global memory. Building for CUDA
