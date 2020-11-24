// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuEventList.h"

#include <algorithm>

namespace gputil
{
EventList::EventList() = default;


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
  : EventList()
{
  for (size_t i = 0; i < event_count; ++i)
  {
    add(events[i]);
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
  clear();
  delete[] extended_;
  extended_ = nullptr;
}


void EventList::add(const Event &event)
{
  if (count_ + 1 >= capacity())
  {
    reserve(std::max(capacity() * 2, capacity() + 1));
  }

  Event *target_array = (extended_) ? extended_ : events_.data();
  target_array[count_++] = event;
}


void EventList::add(std::initializer_list<Event> events)
{
  const size_t new_event_count = events.size();
  if (count_ + new_event_count >= capacity())
  {
    reserve(std::max(capacity() * 2, capacity() + new_event_count));
  }

  Event *target_array = (extended_) ? extended_ : events_.data();
  for (const Event &e : events)
  {
    target_array[count_++] = e;
  }
}


void EventList::add(std::initializer_list<const Event *> events)
{
  const size_t new_event_count = events.size();
  if (count_ + new_event_count >= capacity())
  {
    reserve(std::max(capacity() * 2, capacity() + new_event_count));
  }

  Event *target_array = (extended_) ? extended_ : events_.data();
  for (const Event *e : events)
  {
    target_array[count_++] = *e;
  }
}


void EventList::clear()
{
  Event *target_array = (extended_) ? extended_ : events_.data();
  for (size_t i = 0; i < count_; ++i)
  {
    target_array[i].release();
  }
  count_ = 0;
}


void EventList::reserve(size_t new_size)
{
  if (new_size < capacity_ || !extended_ && new_size < kShortCount)
  {
    // Already large enough.
    return;
  }

  // Lint(KS): only way I can think of to manage this setup.
  auto *new_array = new Event[new_size];  // NOLINT(cppcoreguidelines-owning-memory)
  Event *old_array = (extended_) ? extended_ : events_.data();
  for (size_t i = 0; i < count_; ++i)
  {
    new_array[i] = std::move(old_array[i]);
  }

  delete[] extended_;
  extended_ = new_array;
  capacity_ = new_size;
}
}  // namespace gputil