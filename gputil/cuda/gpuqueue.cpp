// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuqueue.h"

#include "cuda/gpuqueuedetail.h"
#include "cuda/gpueventdetail.h"

#include "gpuapiexception.h"
#include "gputhrow.h"

#include <cuda_runtime.h>

using namespace gputil;

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
    std::function<void (void)> callback;

    inline CallbackWrapper(const std::function<void (void)> &callback)
      : callback(callback)
    {}
  };

  void streamCallback(cudaStream_t event, cudaError_t status, void *userData)
  {
    CallbackWrapper *wrapper = static_cast<CallbackWrapper *>(userData);
    wrapper->callback();
    delete wrapper;
  }
}


Queue::Queue()
  : _queue(nullptr)
{
}


Queue::Queue(Queue &&other)
  : _queue(other._queue)
{
  other._queue = nullptr;
}


Queue::Queue(const Queue &other)
: _queue(nullptr)
{
  _queue = other._queue;
  if (_queue)
  {
    _queue->reference();
  }
}


Queue::Queue(void *platformQueue)
// Note: the platformQueue will be null for the default stream.
  : _queue(new QueueDetail(static_cast<cudaStream_t>(platformQueue), 1, &gputil::destroyStream))
{
}


Queue::~Queue()
{
  if (_queue)
  {
    _queue->release();
  }
  _queue = nullptr;
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
  err = cudaEventRecord(event.detail()->obj(), _queue->obj());
  GPUAPICHECK(err, cudaSuccess, Event());
}


void Queue::flush()
{
  // Not needed.
}


void Queue::finish()
{
  cudaStreamSynchronize(_queue->obj());
}


void Queue::queueCallback(const std::function<void (void)> &callback)
{
  CallbackWrapper *wrapper = nullptr;

  cudaError_t err = cudaSuccess;
  wrapper = new CallbackWrapper(callback);

  err = cudaStreamAddCallback(_queue->obj(), streamCallback, wrapper, 0);

  if (err)
  {
    delete wrapper;
    GPUTHROW2(ApiException(err));
  }
}


QueueDetail *Queue::internal() const
{
  return _queue;
}


Queue &Queue::operator = (const Queue &other)
{
  if (_queue)
  {
    _queue->release();
  }
  _queue = other._queue;
  if (_queue)
  {
    _queue->reference();
  }
  return *this;
}


Queue &Queue::operator = (Queue &&other)
{
  if (_queue)
  {
    _queue->release();
  }

  _queue = other._queue;
  other._queue = nullptr;
  return *this;
}
