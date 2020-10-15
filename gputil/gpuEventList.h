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
  /// Represents a short term object identifying a set of @c Event objects. Generally used to specify a set of events to
  /// wait on before a queued operation may begin. @c EventList objects are intended to be short lived objects, normally
  /// used only as arguments functions enqueuing gpu operations.
  ///
  /// The event list maintains its own reference counting for the added events.
  class gputilAPI EventList
  {
  public:
    /// Number of items the short list. No additional allocation is required until more events are aded.
    static const unsigned kShortCount = 8;

    /// Construct an empty event list.
    EventList();
    /// Construct an event list containing just the given event.
    /// @param event The event to add to the list.
    explicit EventList(const Event &event);
    /// Construct an evet list containing just the given event.
    /// @param event The event to add to the list.
    explicit EventList(const Event *event);
    /// Construct an event list with multiple events.
    /// @param events A pointer to the array of events to add.
    /// @param event_count The number of elements to add from @p events .
    EventList(const Event *events, size_t event_count);
    /// Construct an event list with multiple events.
    /// @param events The list of events to add.
    EventList(std::initializer_list<Event> events);
    /// Construct an event list with multiple events.
    /// @param events The list of events to add.
    EventList(std::initializer_list<const Event *> events);

    /// Destructor - releases any reference counts for events in the list.
    ~EventList();

    /// Request a pointer to the event array. There are @c count() elements.
    /// @return The event array.
    const Event *events() const { return (!extended_) ? events_ : extended_; }

    /// Query the number of events in the list.
    /// @return The number of events in the list.
    inline size_t count() const { return count_; }
    /// An alias of @c count() .
    /// @return The number of events in the list.
    inline size_t size() const { return count_; }
    /// Query the capacity of the event list. Capacity will be extened as required. The minimum capacirty is
    /// @c kShortCount .
    /// @return The available capacity.
    inline size_t capacity() const { return (!extended_) ? kShortCount : capacity_; }

    /// Add an event to the list.
    /// @param event The event to add.
    void add(const Event &event);

    /// An alias for @c add() .
    /// @param event The event to add.
    inline void push_back(const Event &event) { add(event); }  // NOLINT

    /// Add multiple items to the list.
    /// @param events The evnts to add.
    void add(std::initializer_list<Event> events);
    /// Add multiple items to the list.
    /// @param events The evnts to add.
    void add(std::initializer_list<const Event *> events);

    /// Clear the event list, releasing all event references.
    void clear();

    /// Ensure the @c capacity() is at least @c new_size .
    /// @param new_size The event capacity to allow for.
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
}  // namespace gputil

#endif  // GPUEVENTLIST_H
