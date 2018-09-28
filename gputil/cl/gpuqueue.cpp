// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuqueue.h"

#include "cl/gpueventdetail.h"
#include "cl/gpuqueuedetail.h"
#include "gpuapiexception.h"

using namespace gputil;

namespace
{
  struct CallbackWrapper
  {
    std::function<void (void)> callback;

    inline CallbackWrapper(const std::function<void (void)> &callback)
      : callback(callback)
    {}
  };

  inline cl_command_queue q(void *p) { return static_cast<cl_command_queue>(p); }

  void eventCallback(cl_event event, cl_int status, void *userData)
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
  : _queue(new QueueDetail)
{
  _queue->queue = other._queue->queue;
}


Queue::Queue(void *platformQueue)
  : _queue(new QueueDetail)
{
  // Note: This code path takes ownership of an existing reference on the
  // given cl_command_queue object. It does not retain an additional reference.
  cl_command_queue queue = static_cast<cl_command_queue>(platformQueue);
  _queue->queue = queue;
}


Queue::~Queue()
{
  delete _queue;
}


bool Queue::isValid() const
{
  return _queue != nullptr && _queue->queue() != nullptr;
}


void Queue::insertBarrier()
{
  clEnqueueBarrierWithWaitList(_queue->queue(), 0, nullptr, nullptr);
}


Event Queue::mark()
{
  Event event;
  clEnqueueBarrierWithWaitList(_queue->queue(), 0, nullptr, &event.detail()->event);
  return event;
}


void Queue::flush()
{
  clFlush(_queue->queue());
}


void Queue::finish()
{
  clFinish(_queue->queue());
}


void Queue::queueCallback(const std::function<void (void)> &callback)
{
  CallbackWrapper *wrapper = nullptr;
  cl_event barrierEvent;
  cl_int clerr = 0;

  clerr = clEnqueueBarrierWithWaitList(_queue->queue(), 0, nullptr, &barrierEvent);
  GPUAPICHECK2(clerr, CL_SUCCESS);
  wrapper = new CallbackWrapper(callback);
  clerr = clSetEventCallback(barrierEvent, CL_COMPLETE, &eventCallback, wrapper);
  if (clerr)
  {
    delete wrapper;
    GPUTHROW2(ApiException(clerr));
  }
  clerr = clReleaseEvent(barrierEvent);
  GPUAPICHECK2(clerr, CL_SUCCESS);
}


QueueDetail *Queue::internal() const
{
  return _queue;
}


Queue &Queue::operator = (const Queue &other)
{
  if (!_queue)
  {
    _queue = new QueueDetail;
  }
  _queue->queue = other._queue->queue;
  return *this;
}


Queue &Queue::operator = (Queue &&other)
{
  delete _queue;
  _queue = other._queue;
  other._queue = nullptr;
  return *this;
}
