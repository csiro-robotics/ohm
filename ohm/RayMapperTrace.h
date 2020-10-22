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

#include <ohmutil/VectorHash.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <glm/gtc/quaternion.hpp>

#include <unordered_map>
#include <unordered_set>

namespace ohm
{
  class OccupancyMesh;

  /// A @c RayMapper wrapper which adds Third Eye Scene debug visualisation to the process.
  ///
  /// This only functions correctly if ohm is built with the option @c OHM_TES_DEBUG .
  ///
  /// The debug visualisation adds significant overhead. @c ohm::Trace also has to be enabled.
  ///
  /// The visualisation shows rays being integrated, occupied voxels and NDT ellipsoids if present.
  ///
  /// See https://github.com/csiro-robotics/3rdEyeScene
  class ohm_API RayMapperTrace : public RayMapper
  {
  public:
    /// Utility structure tracking the status of a voxel including (optional) ellipsoid shape.
    struct VoxelState
    {
      glm::dquat ellipse_rotation{ 1, 0, 0, 0 };  ///< Rotation applied to a scaled sphere to generate the ellipse.
      glm::dvec3 ellipse_pos{ 0 };                ///< Global position of the voxel ellipse.
      glm::dvec3 ellipse_scale{ 1 };              ///< Ellipse scaling.
      OccupancyType type{ kNull };                ///< Voxel type.
    };

    /// Hash for a @c glm::i16vec4
    struct I16Vec4Hash
    {
      /// Hash operator
      /// @param v Vector to hash
      /// @return 64-bit hash value for v - probably really poor on bucketing.
      inline uint64_t operator()(const glm::i16vec4 &v) const
      {
        return (uint64_t(v.w) << 48) | (uint64_t(v.z) << 32) | (uint64_t(v.y) << 16) | uint64_t(v.x);
      }
    };

    /// Set for tracking voxel state.
    using VoxelMap = std::unordered_map<Key, VoxelState, Key::Hash>;
    /// Tracks touched voxels for NDT sending. We use the region key for XYZ and a W component [0, 7] which identifies
    /// a sector in the region. Each of the first 3 bits selects the lower or upper half of the region with bits
    /// (0, 1, 2) mapping to the (x, y, z) axes.
    ///
    /// Explicitly, the sector mappings are:
    ///
    /// Sector  | X range | Y range | Z range
    /// ------- | ------- | ------- | --------
    /// 0       | lower   | lower   | lower
    /// 1       | upper   | lower   | lower
    /// 2       | lower   | upper   | lower
    /// 3       | upper   | upper   | lower
    /// 4       | lower   | lower   | upper
    /// 5       | upper   | lower   | upper
    /// 6       | lower   | upper   | upper
    /// 7       | upper   | upper   | upper
    using SectorSet = std::unordered_set<glm::i16vec4, I16Vec4Hash>;

    /// Create a trace around the given @p map and mapper. These must outlive this object.
    /// @param map The map the @p true_mapper operates on.
    /// @param true_mapper The wrapped mapper.
    RayMapperTrace(OccupancyMap *map, RayMapper *true_mapper);

    /// Destructor.
    ~RayMapperTrace();

    /// Access the target map.
    /// @return The target map object.
    inline OccupancyMap *map() const { return map_; }
    /// Access the wrapped @c RayMapper .
    /// @return The wrapped mapper.
    inline RayMapper *trueMapper() const { return true_mapper_; }

    /// Validity check - passthrough to the wrapped mapper.
    /// @return True if the wrapped mapper is valid.
    bool valid() const override;

    /// Integrate the given rays into the @c trueMapper() . This also performs debug visualisation around the operation.
    /// @param rays The array of start/end point pairs to integrate.
    /// @param element_count The number of @c glm::dvec3 elements in @p rays, which is twice the ray count.
    /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function.
    size_t integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags = kRfDefault) override;

  private:
    /// Work out the sector key associated with @p key . See @c SectorSet .
    /// @param key The voxel key to translate.
    glm::i16vec4 sectorKey(const Key &key) const;

    /// Cache the initial state of voxels affected by the given @p ray set.
    void cacheState(const glm::dvec3 *rays, size_t element_count, VoxelMap *voxels, SectorSet *regions = nullptr);

    OccupancyMap *map_;
    RayMapper *true_mapper_;
    std::unique_ptr<OccupancyMesh> imp_;
  };
}  // namespace ohm

#endif  // OHM_RAYMAPPERTRACE_H_
