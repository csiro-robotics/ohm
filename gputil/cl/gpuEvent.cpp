// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuEvent.h"

#include "cl/gpuEventDetail.h"

#include "gpuApiException.h"
#include "gpuThrow.h"

namespace gputil
{
Event::Event() = default;

Event::Event(const Event &other)
{
  if (other.imp_)
  {
    imp_ = new EventDetail;  // NOLINT(cppcoreguidelines-owning-memory)
    imp_->event = other.imp_->event;
    clRetainEvent(imp_->event);
  }
}


Event::Event(Event &&other) noexcept
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Event::~Event()
{
  release();
  delete imp_;
}


bool Event::isValid() const
{
  return imp_ && imp_->event;
}


void Event::release()
{
  if (imp_)
  {
    if (imp_->event)
    {
      clReleaseEvent(imp_->event);
      imp_->event = nullptr;
    }
  }
}


bool Event::isComplete() const
{
  if (!isValid())
  {
    return true;
  }

  cl_int status = 0;
  cl_int clerr = clGetEventInfo(imp_->event, CL_EVENT_COMMAND_EXECUTION_STATUS, sizeof(status), &status, nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, false);

  return status == CL_COMPLETE;
}


void Event::wait() const
{
  if (imp_ && imp_->event)
  {
    cl_int clerr = clWaitForEvents(1, &imp_->event);
    GPUAPICHECK2(clerr, CL_SUCCESS);
  }
}


void Event::wait(const Event *events, size_t event_count)
{
  if (event_count)
  {
    auto *events_ocl = static_cast<cl_event *>(alloca(sizeof(cl_event) * event_count));
    cl_uint actual_count = 0;
    for (size_t i = 0; i < event_count; ++i)
    {
      if (events[i].imp_ && events[i].imp_->event)
      {
        events_ocl[actual_count++] = events[i].imp_->event;
      }
    }

    if (actual_count)
    {
      clWaitForEvents(cl_uint(event_count), events_ocl);
    }
  }
}


void Event::wait(const Event **events, size_t event_count)
{
  if (event_count)
  {
    auto *events_ocl = static_cast<cl_event *>(alloca(sizeof(cl_event) * event_count));
    cl_uint actual_count = 0;
    for (size_t i = 0; i < event_count; ++i)
    {
      if (events[i]->imp_ && events[i]->imp_->event)
      {
        events_ocl[actual_count++] = events[i]->imp_->event;
      }
    }

    if (actual_count)
    {
      clWaitForEvents(actual_count, events_ocl);
    }
  }
}


Event &Event::operator=(const Event &other)
{
  if (this != &other)
  {
    release();
    if (other.imp_)
    {
      // Call detail() to ensure _imp is allocated.
      detail()->event = other.imp_->event;
      if (imp_->event)
      {
        clRetainEvent(imp_->event);
      }
    }
  }

  return *this;
}


Event &Event::operator=(Event &&other) noexcept
{
  release();
  if (other.imp_)
  {
    delete imp_;
    imp_ = other.imp_;
    other.imp_ = nullptr;
  }
  return *this;
}


EventDetail *Event::detail()
{
  // Detail requested. Allocate if required.
  if (!imp_)
  {
    imp_ = new EventDetail;  // NOLINT(cppcoreguidelines-owning-memory)
    imp_->event = nullptr;
  }
  return imp_;
}


EventDetail *Event::detail() const
{
  return imp_;
}
}  // namespace gputil
