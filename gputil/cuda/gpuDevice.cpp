// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuDevice.h"

#include "gpuApiException.h"
#include "gpuDeviceDetail.h"
#include "gpuThrow.h"


#include <cuda.h>
#include <cuda_runtime.h>

#include <cstring>
#include <sstream>

using namespace gputil;

Device::Device(bool default_device)
  : imp_(new DeviceDetail)
{
  imp_->device = -1;
  if (default_device)
  {
    cudaError_t err = cudaGetDevice(&imp_->device);
    if (err == cudaSuccess)
    {
      cudaDeviceProp props;
      memset(&props, 0, sizeof(props));
      if (cudaGetDeviceProperties(&props, imp_->device) == cudaSuccess)
      {
        imp_->name = props.name;
      }
    }
    else
    {
      imp_->device = -1;
    }
  }
}


Device::Device(int argc, const char **argv)
  : imp_(new DeviceDetail)
{
  select(argc, argv);
}


Device::Device(const Device &other)
  : imp_(new DeviceDetail)
{
  *imp_ = *other.imp_;
}


Device::Device(Device &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Device::~Device()
{
  delete imp_;
}


const char *Device::name() const
{
  return imp_->name.c_str();
}


const char *Device::info() const
{
  return imp_->info.c_str();
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
  cudaError_t err = cudaGetDevice(&imp_->device);
  if (err == cudaSuccess)
  {
    cudaDeviceProp props;
    memset(&props, 0, sizeof(props));
    if (cudaGetDeviceProperties(&props, imp_->device) == cudaSuccess)
    {
      imp_->name = props.name;
      std::ostringstream str;
      str << "Name: " << props.name << '\n';
      str << "Compute: " << props.major << '.' << props.minor;
      imp_->info = str.str();
    }
  }
  else
  {
    imp_->device = -1;
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
  return imp_ && imp_->device >= 0;
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
  *imp_ = *other.imp_;
  return *this;
}


Device &Device::operator=(Device &&other)
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}
