// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_RAYMAPPERTRACE_H_
#define OHM_RAYMAPPERTRACE_H_

#include "OhmConfig.h"

#include "Key.h"
#include "OccupancyType.h"
#include "RayMapper.h"

#include <glm/vec3.hpp>

#include <glm/gtc/quaternion.hpp>

#include <unordered_map>

namespace ohm
{
  class OccupancyMesh;

  class ohm_API RayMapperTrace : public RayMapper
  {
  public:
    /// Utility structure tracking the status of a voxel including (optional) ellipsoid shape.
    struct VoxelState
    {
      glm::dquat ellipse_rotation{ 1, 0, 0, 0 };
      glm::dvec3 ellipse_pos{ 0 };
      glm::dvec3 ellipse_scale{ 1 };
      OccupancyType type{ kNull };
    };

    /// Set for tracking voxel state.
    using VoxelSet = std::unordered_map<Key, VoxelState, Key::Hash>;

    RayMapperTrace(OccupancyMap *map, RayMapper *true_mapper);

    ~RayMapperTrace();

    bool valid() const override;

    size_t integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags = kRfDefault) override;

  private:
    /// Cache the initial state of voxels affected by the given @p ray set.
    void cacheState(const glm::dvec3 *rays, size_t element_count, VoxelSet *voxels);

    OccupancyMap *map_;
    RayMapper *true_mapper_;
    std::unique_ptr<OccupancyMesh> imp_;
  };
}  // namespace ohm

#endif  // OHM_RAYMAPPERTRACE_H_
