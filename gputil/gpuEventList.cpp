// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuEventList.h"

#include <algorithm>

using namespace gputil;

EventList::EventList()
  : count_(0)
  , capacity_(0)
  , extended_(nullptr)
{
}


EventList::EventList(const Event &event)
  : EventList()
{
  events_[0] = event;
  count_ = 1;
}


EventList::EventList(const Event *event)
  : EventList(*event)
{}


EventList::EventList(const Event *events, size_t event_count)
{
  if (event_count <= kShortCount)
  {
    memcpy(events_, events, sizeof(*events) * event_count);
    count_ = event_count;
  }
  else
  {
    for (size_t i = 0; i < event_count; ++i)
    {
      add(events[i]);
    }
  }
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


EventList::~EventList()
{
}


void EventList::add(const Event &event)
{
  const size_t new_event_count = 1;
  Event *target_array = nullptr;
  if (new_event_count + count_ <= kShortCount)
  {
    target_array = events_;
  }
  else
  {
    reserve(std::max(capacity_ * 2, new_event_count + count_));
    target_array = extended_;

  }

  target_array[count_++] = event;
}


void EventList::add(std::initializer_list<Event> events)
{
  const size_t new_event_count = events.size();
  Event *target_array = nullptr;
  if (new_event_count + count_ <= kShortCount)
  {
    target_array = events_;
  }
  else
  {
    reserve(std::max(capacity_ * 2, new_event_count + count_));
    target_array = extended_;

  }

  for (const Event &e : events)
  {
    target_array[count_++] = e;
  }
}


void EventList::add(std::initializer_list<const Event *> events)
{
  const size_t new_event_count = events.size();
  Event *target_array = nullptr;
  if (new_event_count + count_ <= kShortCount)
  {
    target_array = events_;
  }
  else
  {
    reserve(std::max(capacity_ * 2, new_event_count + count_));
    target_array = extended_;

  }

  for (const Event *e : events)
  {
    target_array[count_++] = *e;
  }
}


void EventList::clear()
{
  Event *target_array = (extended_) ? extended_ : events_;
  for (size_t i = 0; i < count_; ++i)
  {
    target_array[i].release();
  }
  count_ = 0;
}


void EventList::reserve(size_t new_size)
{
  if (new_size < capacity_ || new_size < kShortCount)
  {
    // Already large enough.
    return;
  }

  Event *new_array = new Event[new_size];
  Event *old_array = (extended_) ? extended_ : events_;
  for (size_t i = 0; i < count_; ++i)
  {
    new_array[i] = std::move(old_array[i]);
  }

  delete[] extended_;
  extended_ = new_array;
  capacity_ = new_size;
}
