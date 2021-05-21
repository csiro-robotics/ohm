// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_RAYSQUERYDETAIL_H
#define OHM_RAYSQUERYDETAIL_H

#include "OhmConfig.h"

#include "QueryDetail.h"

#include <ohm/OccupancyType.h>

#include <cmath>

namespace ohm
{
struct ohm_API RaysQueryDetail : QueryDetail
{
  /// Set of origin/end point pairs to lookup in the map.
  std::vector<glm::dvec3> rays_in;
  std::vector<float> unobserved_volumes_out;
  std::vector<OccupancyType> terminal_states_out;
  float volume_coefficient = (float)(1.0 / 180.0 * M_PI * 1.0 / 180.0 * M_PI);
  int occupancy_layer = -1;              ///< Cached occupancy layer index.
  glm::u8vec3 occupancy_dim{ 0, 0, 0 };  ///< Cached occupancy layer voxel dimensions. Voxel mean must exactly match.
  bool valid_layers = false;             ///< Has layer validation passed?
};
}  // namespace ohm

#endif  // OHM_RAYSQUERYDETAIL_H
