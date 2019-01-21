// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMQUERYFLAG_H
#define OHMQUERYFLAG_H

#include "OhmConfig.h"

namespace ohm
{
  // Disable clang-format so it doesn't split the table in the comments below
  // clang-format off

  /// Flags controlling the behaviour of queries.
  ///
  /// Some queries have up to four modes of operation relating to CPU and GPU base calculations.
  /// This relates to where the calculations are performed, CPU or GPU, and whether cached values may be used.
  /// Caching primarily relates to whether data stored in different voxel layers may be used or must be
  /// recalculated. Specific behaviour may vary, but the following table defines the general expected behaviour
  /// based on the flags @c QF_GpuEvaluate and @c QF_NoCache.
  ///
  /// Flags                 | Behaviour
  /// --------------------- | ----------------------------------------------------------------------------------------
  /// Neither               | All query calculations performed in CPU and may used cached or precalculated values.
  /// @c QF_GpuEvaluate     | Invoke the required GPU calculation before collating the results. Cached values may be used and only dirty regions are recalculated.
  /// @c QF_NoCache         | CPU is used to calculate the requested values regardless of whether cached values are available or not.
  /// @c QF_GpuEvaluate, @c QF_NoCache | GPU used to forcibly recalculate affected data even if not dirty. No cached values are used.
  ///
  /// Note that these two flags are essentially mutually exclusive and setting both may result in undefined behaviour.
  enum QueryFlag
  {
    // clang-format on

    /// Zero flag value for completeness. Treat unknown voxels as occupied. Otherwise they are considered free.
    kQfZero = 0,
    kQfUnknownAsOccupied = (1 << 0),
    /// Only report a single result, choosing the closest result.
    kQfNearestResult = (1 << 1),
    /// Request GPU usage for a query.
    kQfGpuEvaluate = (1 << 2),

    /// Do not allow use of cached values. Some queries support using cached data either in the query itself
    /// or in the voxel layout. Setting this flag forces the re-evaluation of data regardless of whether or
    /// not cached data may be up to date. When disabled, queries should recalculate only dirty regions.
    kQfNoCache = (1 << 3),

    /// Report ranges without applying scaling, but calculate with scaling. Used in range based queries.
    kQfReportUnscaledResults = (1 << 4),

    /// Represents the first specialised or non generic query flag. May be extended by specific queries.
    kQfSpecialised = (1 << 16),

    // Legacy flag for backwards compatibility
    kQfGpu = kQfGpuEvaluate,

    /// The default query flags applied.
    kQfDefaultFlags = kQfUnknownAsOccupied
  };
}  // namespace ohm

#endif  // OHMQUERYFLAG_H
