// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuevent.h"

#include "cuda/gpueventdetail.h"

#include "gpuapiexception.h"
#include "gputhrow.h"

#include <cuda.h>
#include <cuda_runtime.h>

using namespace gputil;

namespace gputil
{
  void destroyEvent(cudaEvent_t event)
  {
    if (event)
    {
      cudaError_t err = cudaEventDestroy(event);
      event = nullptr;
      GPUAPICHECK2(err, cudaSuccess);
    }
  }
}

Event::Event()
  : _imp(nullptr)  // created as needed
{

}

Event::Event(const Event &other)
  : _imp(nullptr)  // created as needed
{
  if (other._imp)
  {
    _imp = other._imp;
    _imp->reference();
  }
}


Event::Event(Event &&other)
  : _imp(other._imp)
{
  other._imp = nullptr;
}


Event::~Event()
{
  release();
}


bool Event::isValid() const
{
  return _imp && _imp->obj();
}


void Event::release()
{
  if (_imp)
  {
    _imp->release();
    _imp = nullptr;
  }
}


bool Event::isComplete() const
{
  if (!isValid())
  {
    return true;
  }

  cudaError_t status = cudaEventQuery(_imp->obj());
  switch (status)
  {
  case cudaSuccess:
  case cudaErrorNotReady:
    break;
  default:
    GPUAPICHECK(status, cudaSuccess, true);
    break;
  }

  return status == cudaSuccess;
}


void Event::wait() const
{
  if (_imp && _imp->obj())
  {
    cudaError_t err = cudaEventSynchronize(_imp->obj());
    GPUAPICHECK2(err, cudaSuccess);
  }
}


void Event::wait(const Event *events, size_t eventCount)
{
  for (size_t i = 0; i < eventCount; ++i)
  {
    events[i].wait();
  }
}


void Event::wait(const Event **events, size_t eventCount)
{
  for (size_t i = 0; i < eventCount; ++i)
  {
    events[i]->wait();
  }
}


Event &Event::operator=(const Event &other)
{
  release();
  if (other._imp)
  {
    _imp = other._imp;
    _imp->reference();
  }

  return *this;
}


Event &Event::operator=(Event &&other)
{
  release();
  _imp = other._imp;
  other._imp = nullptr;
  return *this;
}


EventDetail *Event::detail()
{
  // Detail requested. Allocate if required.
  if (!_imp)
  {
    cudaEvent_t event = nullptr;
    cudaError_t err = cudaEventCreateWithFlags(&event, cudaEventBlockingSync);
    GPUAPICHECK(err, cudaSuccess, nullptr);
    _imp = new EventDetail(event, 1, &destroyEvent);
  }
  return _imp;
}
