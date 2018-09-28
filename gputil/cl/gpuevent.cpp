// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuevent.h"

#include "cl/gpueventdetail.h"

#include "gpuapiexception.h"
#include "gputhrow.h"

using namespace gputil;

Event::Event()
  : _imp(nullptr)  // created as needed
{

}

Event::Event(const Event &other)
  : _imp(nullptr)  // created as needed
{
  if (other._imp)
  {
    _imp = new EventDetail;
    _imp->event = other._imp->event;
    clRetainEvent(_imp->event);
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
  delete _imp;
}


bool Event::isValid() const
{
  return _imp && _imp->event;
}


void Event::release()
{
  if (_imp)
  {
    if (_imp->event)
    {
      clReleaseEvent(_imp->event);
      _imp->event = nullptr;
    }
  }
}


bool Event::isComplete() const
{
  if (!isValid())
  {
    return true;
  }

  cl_int status;
  cl_int clerr = clGetEventInfo(_imp->event, CL_EVENT_COMMAND_EXECUTION_STATUS, sizeof(status), &status, nullptr);

  GPUAPICHECK(clerr, CL_SUCCESS, false);

  return status == CL_COMPLETE;
}


void Event::wait() const
{
  if (_imp && _imp->event)
  {
    cl_int clerr = clWaitForEvents(1, &_imp->event);
    GPUAPICHECK2(clerr, CL_SUCCESS);
  }
}


void Event::wait(const Event *events, size_t eventCount)
{
  if (eventCount)
  {
    cl_event *eventsOcl = (cl_event *)alloca(sizeof(cl_event) * eventCount);
    cl_uint actualCount = 0;
    for (size_t i = 0; i < eventCount; ++i)
    {
      if (events[i]._imp && events[i]._imp->event)
      {
        eventsOcl[actualCount++] = events[i]._imp->event;
      }
    }

    if (actualCount)
    {
      clWaitForEvents((cl_uint)eventCount, eventsOcl);
    }
  }
}


void Event::wait(const Event **events, size_t eventCount)
{
  if (eventCount)
  {
    cl_event *eventsOcl = (cl_event *)alloca(sizeof(cl_event) * eventCount);
    cl_uint actualCount = 0;
    for (size_t i = 0; i < eventCount; ++i)
    {
      if (events[i]->_imp && events[i]->_imp->event)
      {
        eventsOcl[actualCount++] = events[i]->_imp->event;
      }
    }

    if (actualCount)
    {
      clWaitForEvents(actualCount, eventsOcl);
    }
  }
}


Event &Event::operator=(const Event &other)
{
  release();
  if (other._imp)
  {
    // Call detail() to ensure _imp is allocated.
    detail()->event = other._imp->event;
    if (_imp->event)
    {
      clRetainEvent(_imp->event);
    }
  }

  return *this;
}


Event &Event::operator=(Event &&other)
{
  release();
  if (other._imp)
  {
    delete _imp;
    _imp = other._imp;
    other._imp = nullptr;
  }
  return *this;
}


EventDetail *Event::detail()
{
  // Detail requested. Allocate if required.
  if (!_imp)
  {
    _imp = new EventDetail;
    _imp->event = nullptr;
  }
  return _imp;
}


EventDetail *Event::detail() const
{
  return _imp;
}
