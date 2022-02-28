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

namespace gputil
{
void destroyStream(cudaStream_t &stream)
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

  explicit inline CallbackWrapper(std::function<void(void)> callback)
    : callback(std::move(callback))
  {}
};

void streamCallback(cudaStream_t /*event*/, cudaError_t /*status*/, void *user_data)
{
  auto *wrapper = static_cast<CallbackWrapper *>(user_data);
  wrapper->callback();
  // Lint(KS): No RAII option available for this
  delete wrapper;  // NOLINT(cppcoreguidelines-owning-memory)
}


Queue::Queue()
  : queue_(new QueueDetail())
{}


Queue::Queue(Queue &&other) noexcept
  : queue_(std::exchange(other.queue_, nullptr))
{}


Queue::Queue(const Queue &other)
  : queue_(other.queue_)
{}


Queue::Queue(void *platform_queue)
  : queue_(new QueueDetail())
{
  // Note: the platform_queue will be null for the default stream.
  queue_->queue = static_cast<cudaStream_t>(platform_queue);
}


Queue::~Queue() = default;


bool Queue::isValid() const
{
  return queue_ != nullptr;
}


void Queue::insertBarrier()
{
  // Nothing to do for CUDA. A single stream is implicitly sequential.
}


Event Queue::mark()
{
  Event event;
  cudaError_t err = cudaSuccess;
  err = cudaEventRecord(event.detail()->obj(), queue_->queue);
  GPUAPICHECK(err, cudaSuccess, Event());
  return event;
}


void Queue::setSynchronous(bool synchronous)
{
  queue_->force_synchronous = synchronous;
}


bool Queue::synchronous() const
{
  return queue_->force_synchronous;
}


void Queue::flush()
{
  // Not needed.
}


void Queue::finish()
{
  cudaError_t err = cudaStreamSynchronize(queue_->queue);
  GPUAPICHECK2(err, cudaSuccess);
}


void Queue::queueCallback(const std::function<void(void)> &callback)
{
  CallbackWrapper *wrapper = nullptr;

  cudaError_t err = cudaSuccess;
  // Lint(KS): No nice RAII alternative for this
  wrapper = new CallbackWrapper(callback);  // NOLINT(cppcoreguidelines-owning-memory)

  err = cudaStreamAddCallback(queue_->queue, streamCallback, wrapper, 0);

  if (err)
  {
    // Lint(KS): No nice RAII alternative for this
    delete wrapper;  // NOLINT(cppcoreguidelines-owning-memory)
    GPUTHROW2(ApiException(err, nullptr, __FILE__, __LINE__));
  }
}


QueueDetail *Queue::internal() const
{
  return queue_.get();
}


Queue &Queue::operator=(const Queue &other)
{
  if (this != &other)
  {
    queue_ = other.queue_;
  }
  return *this;
}


Queue &Queue::operator=(Queue &&other) noexcept
{
  queue_ = std::move(other.queue_);
  return *this;
}
}  // namespace gputil
