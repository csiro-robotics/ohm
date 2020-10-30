// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuQueue.h"

#include "gpuApiException.h"

#include "cl/gpuEventDetail.h"
#include "cl/gpuQueueDetail.h"
#include "gpuApiException.h"

using namespace gputil;

namespace
{
struct CallbackWrapper
{
  std::function<void(void)> callback;

  inline CallbackWrapper(const std::function<void(void)> &callback)
    : callback(callback)
  {}
};

// inline cl_command_queue q(void *p) { return static_cast<cl_command_queue>(p); }

void eventCallback(cl_event /*event*/, cl_int /*status*/, void *user_data)
{
  CallbackWrapper *wrapper = static_cast<CallbackWrapper *>(user_data);
  wrapper->callback();
  delete wrapper;
}
}  // namespace

Queue::Queue()
  : queue_(nullptr)
{}


Queue::Queue(Queue &&other) noexcept
  : queue_(other.queue_)
{
  other.queue_ = nullptr;
}


Queue::Queue(const Queue &other)
  : queue_(new QueueDetail)
{
  queue_->queue = other.queue_->queue;
}


Queue::Queue(void *platform_queue)
  : queue_(new QueueDetail)
{
  // Note: This code path takes ownership of an existing reference on the
  // given cl_command_queue object. It does not retain an additional reference.
  cl_command_queue queue = static_cast<cl_command_queue>(platform_queue);
  queue_->queue = queue;
}


Queue::~Queue()
{
  delete queue_;
}


bool Queue::isValid() const
{
  return queue_ != nullptr && queue_->queue() != nullptr;
}


void Queue::insertBarrier()
{
  clEnqueueBarrierWithWaitList(queue_->queue(), 0, nullptr, nullptr);
}


Event Queue::mark()
{
  Event event;
  clEnqueueBarrierWithWaitList(queue_->queue(), 0, nullptr, &event.detail()->event);
  return event;
}


void Queue::flush()
{
  clFlush(queue_->queue());
}


void Queue::finish()
{
  clFinish(queue_->queue());
}


void Queue::queueCallback(const std::function<void(void)> &callback)
{
  CallbackWrapper *wrapper = nullptr;
  cl_event barrier_event;
  cl_int clerr = 0;

  clerr = clEnqueueBarrierWithWaitList(queue_->queue(), 0, nullptr, &barrier_event);
  GPUAPICHECK2(clerr, CL_SUCCESS);
  wrapper = new CallbackWrapper(callback);
  clerr = clSetEventCallback(barrier_event, CL_COMPLETE, &eventCallback, wrapper);
  if (clerr)
  {
    delete wrapper;
    GPUTHROW2(ApiException(clerr));
  }
  clerr = clReleaseEvent(barrier_event);
  GPUAPICHECK2(clerr, CL_SUCCESS);
}


QueueDetail *Queue::internal() const
{
  return queue_;
}


Queue &Queue::operator=(const Queue &other)
{
  if (!queue_)
  {
    queue_ = new QueueDetail;
  }
  queue_->queue = other.queue_->queue;
  return *this;
}


Queue &Queue::operator=(Queue &&other) noexcept
{
  delete queue_;
  queue_ = other.queue_;
  other.queue_ = nullptr;
  return *this;
}
