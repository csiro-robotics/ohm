// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUTILEVENT_H
#define GPUTILEVENT_H

#include "gpuConfig.h"

#include <cstddef>

namespace gputil
{
  struct EventDetail;

  /// Marks a point in the execution stream on which we can block and await completion.
  ///
  /// Note: this is analogous to an OpenCL event. CUDA does not have as precise a parallel, supporting only an
  /// event concept which blocks an entire stream awaiting event completion. However, CUDA streams are inherently
  /// more serial, awaiting completion of each command, whereas OpenCL queues start execution of each item without
  /// waiting for the previous item unless an events are used.
  class gputilAPI Event
  {
  public:
    Event();
    Event(const Event &other);
    Event(Event &&other) noexcept;

    ~Event();

    /// Is this a valid event?
    bool isValid() const;

    /// Release the underlying event, invalidating this object.
    void release();

    /// Has this event completed. Note that invalid events always return @c true.
    /// @return True if this event is invalid or has completed execution.
    bool isComplete() const;

    /// Block until this event to completes.
    void wait() const;

    /// Block the CPU on multiple events before continuing.
    ///
    /// An overload accepts an array of pointers.
    ///
    /// @param events Array of events to wait on.
    /// @param event_count The number of elements in @p events.
    static void wait(const Event *events, size_t event_count);

    /// @overload
    static void wait(const Event **events, size_t event_count);

    Event &operator=(const Event &other);
    Event &operator=(Event &&other) noexcept;

    EventDetail *detail();
    EventDetail *detail() const;

  private:
    EventDetail *imp_;
  };
}

#endif // GPUTILEVENT_H

