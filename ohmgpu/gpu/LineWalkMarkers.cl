// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LINE_WALK_MARKERS_CL
#define LINE_WALK_MARKERS_CL

/// Flags affecting line walking
typedef enum LineWalkFlag_t
{
  /// No options set: default behaviour.
  kLineWalkFlagNone = 0,
  /// Option for reverse line walking. The line walks the segment in reverse.
  kLineWalkFlagReverse = (1 << 0),
  /// Option to force reporting the end voxel last in a reverse walk. This occurs normally when @p kLineWalkFlagReverse
  /// is not set.
  kLineWalkFlagForReportEndLast = (1 << 1)
} LineWalkFlag;

#endif  // LINE_WALK_MARKERS_CL
