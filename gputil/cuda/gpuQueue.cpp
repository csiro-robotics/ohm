// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuQueue.h"

#include "gputil/cuda/gpuEventDetail.h"
#include "gputil/cuda/gpuQueueDetail.h"

#include "gputil/gpuApiException.h"
#include "gputil/gpuThrow.h"

#include <cuda_runtime.h>

using namespace gputil;

namespace gputil
{
void destroyStream(cudaStream_t &stream)  // NOLINT(google-runtime-references)
{
  if (stream)
  {
    cudaError_t err = cudaStreamDestroy(stream);
    stream = nullptr;
    GPUAPICHECK2(err, cudaSuccess);
  }
}

struct CallbackWrapper
{
  std::function<void(void)> callback;

  inline CallbackWrapper(const std::function<void(void)> &callback)
    : callback(callback)
  {}
};

void streamCallback(cudaStream_t /*event*/, cudaError_t /*status*/, void *user_data)
{
  CallbackWrapper *wrapper = static_cast<CallbackWrapper *>(user_data);
  wrapper->callback();
  delete wrapper;
}
}  // namespace gputil


Queue::Queue()
  : queue_(nullptr)
{}


Queue::Queue(Queue &&other) noexcept
  : queue_(other.queue_)
{
  other.queue_ = nullptr;
}


Queue::Queue(const Queue &other)
  : queue_(nullptr)
{
  queue_ = other.queue_;
  if (queue_)
  {
    queue_->reference();
  }
}


Queue::Queue(void *platform_queue)
  // Note: the platform_queue will be null for the default stream.
  : queue_(new QueueDetail(static_cast<cudaStream_t>(platform_queue), 1, &gputil::destroyStream))
{}


Queue::~Queue()
{
  if (queue_)
  {
    queue_->release();
  }
  queue_ = nullptr;
}


bool Queue::isValid() const
{
  return false;
}


void Queue::insertBarrier()
{
  // Nothing to do for CUDA. A single stream is implicitly sequential.
}


Event Queue::mark()
{
  Event event;
  cudaError_t err;
  err = cudaEventRecord(event.detail()->obj(), queue_->obj());
  GPUAPICHECK(err, cudaSuccess, Event());
  return event;
}


void Queue::flush()
{
  // Not needed.
}


void Queue::finish()
{
  cudaError_t err = cudaStreamSynchronize(queue_->obj());
  GPUAPICHECK2(err, cudaSuccess);
}


void Queue::queueCallback(const std::function<void(void)> &callback)
{
  CallbackWrapper *wrapper = nullptr;

  cudaError_t err = cudaSuccess;
  wrapper = new CallbackWrapper(callback);

  err = cudaStreamAddCallback(queue_->obj(), streamCallback, wrapper, 0);

  if (err)
  {
    delete wrapper;
    GPUTHROW2(ApiException(err));
  }
}


QueueDetail *Queue::internal() const
{
  return queue_;
}


Queue &Queue::operator=(const Queue &other)
{
  if (queue_)
  {
    queue_->release();
    queue_ = nullptr;
  }
  if (other.queue_)
  {
    queue_ = other.queue_;
    queue_->reference();
  }
  return *this;
}


Queue &Queue::operator=(Queue &&other) noexcept
{
  if (queue_)
  {
    queue_->release();
  }

  queue_ = other.queue_;
  other.queue_ = nullptr;
  return *this;
}
