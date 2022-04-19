// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LINE_WALK_MARKERS_CL
#define LINE_WALK_MARKERS_CL

/// Flags affecting line walking
typedef enum LineWalkFlag
{
  /// No options set: default behaviour.
  kLineWalkFlagNone = 0u,
  /// Skip reporting the voxel containing the start point if different from the end point?
  kExcludeStartVoxel = (1u << 0u),
  /// Skip reporting the voxel containing the end point if different from the start point.
  kExcludeEndVoxel = (1u << 1u),
  /// Option for reverse line walking. The line walks the segment in reverse.
  kLineWalkFlagReverse = (1u << 2u),
  /// Option to force reporting the end voxel last in a reverse walk. This occurs normally when @p kLineWalkFlagReverse
  /// is not set.
  kLineWalkFlagForReportEndLast = (1u << 3u)
} LineWalkFlag;

#endif  // LINE_WALK_MARKERS_CL
