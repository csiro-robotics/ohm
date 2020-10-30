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
  /// Construct a null/invalid event.
  Event();
  /// Copy constructor. Refers to the same GPU API event as @p other .
  /// @param other Even to copy.
  Event(const Event &other);
  /// Move constructor.
  /// @param other Even to move.
  Event(Event &&other) noexcept;

  /// Destructor - releases GPU API event.
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

  /// Copy assignment. Refers to the same GPU API event as @p other .
  /// @param other Even to copy.
  /// @return `*this`
  Event &operator=(const Event &other);
  /// Move assignment.
  /// @param other Even to move.
  /// @return `*this`
  Event &operator=(Event &&other) noexcept;

  /// Get the internal event representation - constructed if null.
  /// @return The internal event detail.
  EventDetail *detail();
  /// Get the internal event representation.
  /// @return The internal event detail - may be null.
  EventDetail *detail() const;

private:
  EventDetail *imp_;
};
}  // namespace gputil

#endif  // GPUTILEVENT_H
