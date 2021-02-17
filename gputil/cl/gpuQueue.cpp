// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuQueue.h"

#include "gpuApiException.h"

#include "cl/gpuEventDetail.h"
#include "cl/gpuQueueDetail.h"

#include <clu/clu.h>

namespace gputil
{
namespace
{
struct CallbackWrapper
{
  std::function<void(void)> callback;

  explicit inline CallbackWrapper(std::function<void(void)> callback)
    : callback(std::move(callback))
  {}
};

// inline cl_command_queue q(void *p) { return static_cast<cl_command_queue>(p); }

void eventCallback(cl_event /*event*/, cl_int /*status*/, void *user_data)
{
  auto *wrapper = static_cast<CallbackWrapper *>(user_data);
  wrapper->callback();
  // Lint(KS): No RAII option available for this
  delete wrapper;  // NOLINT(cppcoreguidelines-owning-memory)
}
}  // namespace

Queue::Queue() = default;


Queue::Queue(Queue &&other) noexcept
  : queue_(std::exchange(other.queue_, nullptr))
{}


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
  // Lint: linting suggest using `auto` for the type below, but cl_command_queue is a pointer typedef so it would
  // need to be auto *. I find this misleading becuase cl_command_queue is not a pointer type.
  cl_command_queue queue = static_cast<cl_command_queue>(platform_queue);  // NOLINT(modernize-use-auto)
  queue_->queue = queue;
}


Queue::~Queue() = default;


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
  cl_event barrier_event{};
  cl_int clerr = 0;

  clerr = clEnqueueBarrierWithWaitList(queue_->queue(), 0, nullptr, &barrier_event);
  GPUAPICHECK2(clerr, CL_SUCCESS);
  // Lint(KS): No nice RAII alternative for this
  wrapper = new CallbackWrapper(callback);  // NOLINT(cppcoreguidelines-owning-memory)
  clerr = clSetEventCallback(barrier_event, CL_COMPLETE, &eventCallback, wrapper);
  if (clerr)
  {
    // Lint(KS): No nice RAII alternative for this
    delete wrapper;  // NOLINT(cppcoreguidelines-owning-memory)
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
  if (this != &other)
  {
    if (!queue_)
    {
      queue_ = new QueueDetail;
    }
    queue_->queue = other.queue_->queue;
  }
  return *this;
}


Queue &Queue::operator=(Queue &&other) noexcept
{
  queue_ = std::move(other.queue_);
  return *this;
}
}  // namespace gputil
