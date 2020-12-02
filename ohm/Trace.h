// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef TRACE_H
#define TRACE_H

#include "OhmConfig.h"

#include <string>

namespace ohm
{
/// Debug visaulisation trace categories.
enum TraceCategory : uint16_t
{
  kTcDefault,      ///< Default category (catchall)
  kTcMap,          ///< Parent category for occupancy map generation.
  kTcRays,         ///< Ray insertion
  kTcVoxels,       ///< Voxel state
  kTcNdt,          ///< NDT shapes
  kTcHeightmap,    ///< Parent category for heightmap generation
  kTcHmVoxel,      ///< Heightmap voxel
  kTcHmClearance,  ///< Heightmap clearance
  kTcHmVisit,      ///< Heightmap visit/open lists
  kTcHmInfo,       ///< Heightmap info text
};

namespace trace
{
/// Initialise the trace system. This enables debug rendering connections via 3rd Eye Scene.
///
/// Requires `TES_ENABLE` cmake option to be on.
///
/// @todo Allow the server reference frame to be specified: default is right handled, XYZ = rigth, forward, up.
///
/// @param file_stream When set, opens a file stream to record trace to this file.
void ohm_API init(const std::string &file_stream = std::string());

/// Finalise the trace system.
void ohm_API done();
}  // namespace trace

/// A utility class which calls @c trace::init() and @c trace::done() on construction and destruction respectively.
/// This supports stack based trace initialisation and shutdown.
class ohm_API Trace
{
public:
  /// Constructor: calls through to @c trace::init()
  inline Trace(const std::string &file_stream)  // NOLINT(google-explicit-constructor)
  {
    trace::init(file_stream);
  }
  /// Destructor: calls through to @c trace::done()
  inline ~Trace() { trace::done(); }

  Trace(const Trace &) = delete;
  Trace &operator=(const Trace &) = delete;
};
}  // namespace ohm

#endif  // TRACE_H
