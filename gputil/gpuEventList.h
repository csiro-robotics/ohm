// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTLIST_H
#define GPUEVENTLIST_H

#include "gpuConfig.h"

#include "gpuEvent.h"

#include <initializer_list>

namespace gputil
{
  /// Represents a short term object identifying a set of @c Event objects. Generally used
  /// to specify a set of events to wait on before a queued operation may begin. @c EventList
  /// objects are intended to be short lived objects, normally used only as arguments functions
  /// enqueuing gpu operations.
  class gputilAPI EventList
  {
  public:
    static const unsigned kShortCount = 8;

    EventList();
    explicit EventList(const Event &event);
    explicit EventList(const Event *event);
    EventList(const Event *events, size_t event_count);
    EventList(std::initializer_list<Event> events);
    EventList(std::initializer_list<const Event *> events);

    ~EventList();

    const Event *events() const { return events_; }
    inline size_t count() const { return count_; }
    inline size_t size() const { return count_; }

    void add(const Event &event);
    inline void push_back(const Event &event) { add(event); } // NOLINT

    void add(std::initializer_list<Event> events);
    void add(std::initializer_list<const Event *> events);

    void clear();

    void reserve(size_t new_size);

  private:
    /// Event short list. Only valid when _count <= ShortCount
    Event events_[kShortCount];
    /// Number of events in the list.
    size_t count_;
    /// Capacity of _extended. Only used when _extended is valid.
    size_t capacity_;
    /// Extended list. Allocated when the number of events is ShortCount or greater.
    Event *extended_;
  };
}

#endif // GPUEVENTLIST_H
