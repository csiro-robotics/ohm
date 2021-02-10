// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEWALKVISITMODE_H
#define OHM_PLANEWALKVISITMODE_H

#include "OhmConfig.h"

namespace ohm
{
/// Mode selection for handling neighbours in various @c PlaneWalker implementations used by @c Heightmap .
/// Determines now neigbhours are handled when visiting a node.
enum class PlaneWalkVisitMode
{
  /// Do nothing with neighbours. Not added.
  kIgnoreNeighbours,
  /// Add (planar) neighbours for processing.
  kAddUnvisitedNeighbours,
  /// Add (planar) neighbours for processing as long as they have not yet been visited in a 2D sense.
  ///
  /// This option should be used with @c PlaneFillLayeredWalk when a column cannot find a valid ground candidate. The
  /// layered fill can then traverse unobserved space.
  kAddUnvisitedColumnNeighbours
};
}  // namespace ohm

#endif  // OHM_PLANEWALKVISITMODE_H
