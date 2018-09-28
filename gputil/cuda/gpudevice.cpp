// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpudevice.h"

#include "gpuapiexception.h"
#include "gpudevicedetail.h"
#include "gputhrow.h"


#include <cuda.h>
#include <cuda_runtime.h>

#include <cstring>
#include <sstream>

using namespace gputil;

Device::Device(bool defaultDevice)
  : _imp(new DeviceDetail)
{
  _imp->device = -1;
  if (defaultDevice)
  {
    cudaError_t err = cudaGetDevice(&_imp->device);
    if (err == cudaSuccess)
    {
      cudaDeviceProp props;
      memset(&props, 0, sizeof(props));
      if (cudaGetDeviceProperties(&props, _imp->device) == cudaSuccess)
      {
        _imp->name = props.name;
      }
    }
    else
    {
      _imp->device = -1;
    }
  }
}


Device::Device(int argc, const char **argv)
  : _imp(new DeviceDetail)
{
  select(argc, argv);
}


Device::Device(const Device &other)
  : _imp(new DeviceDetail)
{
  *_imp = *other._imp;
}


Device::Device(Device &&other)
  : _imp(other._imp)
{
  other._imp = nullptr;
}


Device::~Device()
{
  delete _imp;
}


const char *Device::name() const
{
  return _imp->name.c_str();
}


const char *Device::info() const
{
  return _imp->info.c_str();
}


Queue Device::defaultQueue() const
{
  return Queue(nullptr);
}


Queue Device::createQueue(unsigned flags) const
{
  cudaStream_t stream = nullptr;
  cudaError_t err = cudaStreamCreate(&stream);
  GPUAPICHECK(err, cudaSuccess, Queue());
  return Queue(stream);
}


bool Device::select(int argc, const char **argv)
{
  // TODO: use argv
  cudaError_t err = cudaGetDevice(&_imp->device);
  if (err == cudaSuccess)
  {
    cudaDeviceProp props;
    memset(&props, 0, sizeof(props));
    if (cudaGetDeviceProperties(&props, _imp->device) == cudaSuccess)
    {
      _imp->name = props.name;
      std::ostringstream str;
      str << "Name: " << props.name << '\n';
      str << "Compute: " << props.major << '.' << props.minor;
      _imp->info = str.str();
    }
  }
  else
  {
    _imp->device = -1;
  }

  return isValid();
}


void Device::setDebugGpu(bool)
{
  // Ignored for CUDA.
}


bool Device::debugGpu() const
{
  return false;
}


void Device::addSearchPath(const char *)
{
  // Ignored for CUDA.
}


const char *Device::searchPaths() const
{
  return "";
}


bool Device::isValid() const
{
  return _imp && _imp->device >= 0;
}


uint64_t Device::deviceMemory() const
{
  size_t free = 0;
  size_t total = 0;
  cudaError_t err = cudaMemGetInfo(&free, &total);
  GPUAPICHECK(err, cudaSuccess, 0);
  return total;
}


Device &Device::operator=(const Device &other)
{
  *_imp = *other._imp;
  return *this;
}


Device &Device::operator=(Device &&other)
{
  delete _imp;
  _imp = other._imp;
  other._imp = nullptr;
  return *this;
}
