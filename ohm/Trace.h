// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef TRACE_H
#define TRACE_H

#include "OhmConfig.h"

namespace ohm
{
namespace trace
{
/// Initialise the trace system. This enables debug rendering connections via 3rd Eye Scene.
///
/// Requires `TES_ENABLE` cmake option to be on.
///
/// @todo Allow the server reference frame to be specified: default is right handled, XYZ = rigth, forward, up.
///
/// @param file_stream When set, opens a file stream to record trace to this file.
void ohm_API init(const char *file_stream = nullptr);

/// Finalise the trace system.
void ohm_API done();
}  // namespace trace

/// A utility class which calls @c trace::init() and @c trace::done() on construction and destruction respectively.
/// This supports stack based trace initialisation and shutdown.
class ohm_API Trace
{
public:
  /// Constructor: calls through to @c trace::init()
  inline Trace(const char *file_stream) { trace::init(file_stream); }
  /// Destructor: calls through to @c trace::done()
  inline ~Trace() { trace::done(); }
};
}  // namespace ohm

#endif  // TRACE_H
