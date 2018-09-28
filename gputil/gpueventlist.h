// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTLIST_H_
#define GPUEVENTLIST_H_

#include "gpuconfig.h"

#include "gpuevent.h"

#include <cstdio>
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
    static const unsigned ShortCount = 8;

    EventList();
    explicit EventList(const Event &event);
    explicit EventList(const Event *event);
    EventList(std::initializer_list<Event> events);
    EventList(std::initializer_list<const Event *> events);

    ~EventList();

    const Event *events() const;
    inline size_t count() const { return _count; }
    inline size_t size() const { return _count; }

    void add(const Event &event);
    inline void push_back(const Event &event) { add(event); }

    void add(std::initializer_list<Event> events);
    void add(std::initializer_list<const Event *> events);

    void clear();

    void reserve(size_t newSize);

  private:
    /// Event short list. Only valid when _count <= ShortCount
    Event _events[ShortCount];
    /// Number of events in the list.
    size_t _count;
    /// Capacity of _extended. Only used when _extended is valid.
    size_t _capacity;
    /// Extended list. Allocated when the number of events is ShortCount or greater.
    Event *_extended;
  };
}

#endif // GPUEVENTLIST_H_
