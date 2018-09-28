// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpueventlist.h"

#include <algorithm>

using namespace gputil;

EventList::EventList()
  : _count(0)
  , _capacity(0)
  , _extended(nullptr)
{
}


EventList::EventList(const Event &event)
  : EventList()
{
  _events[0] = event;
  _count = 1;
}


EventList::EventList(const Event *event)
  : EventList(*event)
{
}


EventList::EventList(std::initializer_list<Event> events)
  : EventList()
{
  add(events);
}


EventList::EventList(std::initializer_list<const Event *> events)
  : EventList()
{
  add(events);
}


void EventList::add(std::initializer_list<Event> events)
{
  const size_t newEventCount = events.size();
  Event *targetArray = nullptr;
  if (newEventCount + _count <= ShortCount)
  {
    targetArray = _events;
  }
  else
  {
    reserve(std::max(_capacity * 2, newEventCount + _count));
    targetArray = _extended;

  }

  for (const Event &e : events)
  {
    targetArray[_count++] = e;
  }
}


void EventList::add(std::initializer_list<const Event *> events)
{
  const size_t newEventCount = events.size();
  Event *targetArray = nullptr;
  if (newEventCount + _count <= ShortCount)
  {
    targetArray = _events;
  }
  else
  {
    reserve(std::max(_capacity * 2, newEventCount + _count));
    targetArray = _extended;

  }

  for (const Event *e : events)
  {
    targetArray[_count++] = *e;
  }
}


void EventList::clear()
{
  Event *targetArray = (_extended) ? _extended : _events;
  for (size_t i = 0; i < _count; ++i)
  {
    targetArray[i].release();
  }
  _count = 0;
}


void EventList::reserve(size_t newSize)
{
  if (newSize < _capacity || newSize < ShortCount)
  {
    // Already large enough.
    return;
  }

  Event *newArray = new Event[newSize];
  Event *oldArray = (_extended) ? _extended : _events;
  for (size_t i = 0; i < _count; ++i)
  {
    newArray[i] = std::move(oldArray[i]);
  }

  delete[] _extended;
  _extended = newArray;
  _capacity = newSize;
}
