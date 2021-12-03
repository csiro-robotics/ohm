// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmConfig.h"

namespace ohm
{
class OccupancyMap;

/// Debug draw the @p map using 3rd Eye Scene (if available).
///
/// Requires @c ohm::trace::available()
/// @param map The map to draw.
/// @return True if a 3es is available and a connection is present.
bool ohm_API debugDraw(const ohm::OccupancyMap &map);

/// Debug draw @p map via 3es with Ndt TM data. Fails if required layers are not available.
/// @param map The map to draw.
/// @return True if a 3es is available and a connection is present.
bool ohm_API debugDrawNdtTm(const ohm::OccupancyMap &map);

/// Debug draw @p map via 3es with Ndt occupancy data. Fails if required layers are not available.
/// @param map The map to draw.
/// @return True if a 3es is available and a connection is present.
bool ohm_API debugDrawNdtOm(const ohm::OccupancyMap &map);

/// Debug draw @p map via 3es using only occupancy and voxel mean data. Fails if occupancy layer is not available.
/// @param map The map to draw.
/// @return True if a 3es is available and a connection is present.
bool ohm_API debugDrawOccupancy(const ohm::OccupancyMap &map);
}  // namespace ohm
