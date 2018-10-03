// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuEvent.h"

#include "cuda/gpuEventDetail.h"

#include "gpuApiException.h"
#include "gpuThrow.h"

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
  : imp_(nullptr)  // created as needed
{

}

Event::Event(const Event &other)
  : imp_(nullptr)  // created as needed
{
  if (other.imp_)
  {
    imp_ = other.imp_;
    imp_->reference();
  }
}


Event::Event(Event &&other)
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Event::~Event()
{
  release();
}


bool Event::isValid() const
{
  return imp_ && imp_->obj();
}


void Event::release()
{
  if (imp_)
  {
    imp_->release();
    imp_ = nullptr;
  }
}


bool Event::isComplete() const
{
  if (!isValid())
  {
    return true;
  }

  cudaError_t status = cudaEventQuery(imp_->obj());
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
  if (imp_ && imp_->obj())
  {
    cudaError_t err = cudaEventSynchronize(imp_->obj());
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
  if (other.imp_)
  {
    imp_ = other.imp_;
    imp_->reference();
  }

  return *this;
}


Event &Event::operator=(Event &&other)
{
  release();
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


EventDetail *Event::detail()
{
  // Detail requested. Allocate if required.
  if (!imp_)
  {
    cudaEvent_t event = nullptr;
    cudaError_t err = cudaEventCreateWithFlags(&event, cudaEventBlockingSync);
    GPUAPICHECK(err, cudaSuccess, nullptr);
    imp_ = new EventDetail(event, 1, &destroyEvent);
  }
  return imp_;
}
