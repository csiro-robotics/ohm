//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPER_H
#define RAYMAPPER_H

#include "OhmConfig.h"

#include "CalculateSegmentKeys.h"
#include "KeyList.h"
#include "MapCache.h"
#include "RayFilter.h"
#include "RayFlag.h"
#include "RayMapper.h"
#include "RayMapperInterface.h"
#include "Voxel.h"

#include <glm/vec3.hpp>

namespace ohm
{
  /// A @c RayMapper implementation built around updating a map in CPU. Primarily this deals with updating the
  /// @c OccupancyMap class. The use of templates also allows for the update of an @c NdtMap .
  ///
  /// The @c integrateRays() implementation performs a single threaded calculation of the voxels to update and touches
  /// those voxels one at a time. The @c MAPINTERFACE type is used to access required information from the @c MAP and
  /// update individual voxels.
  ///
  /// The @c MAPINTERFACE must support the following interface, where @c MAP corresponds to the @c RayMapperCpu template
  /// type of the same name:
  ///
  /// ```
  /// struct MapInterface
  /// {
  ///   /// Extract an @c OccupancyMap from the @c MAP type (mutable).
  ///   static OccupancyMap &occupancyMap(MAP &map) ;
  ///   /// Extract an @c OccupancyMap from the @c MAP type (const).
  ///   static const OccupancyMap &occupancyMap(const MAP &map);
  ///   /// Retrieve the active @c RayFilterFunction for the given @c map . May be empty.
  ///   static RayFilterFunction rayFilter(const MAP &map);
  ///   /// Query the voxel key within @p map for the given @p point .
  ///   static Key voxelKey(MAP &map, const glm::dvec3 &point);
  ///   /// Fetch a mutable @c Voxel for the given @p Key in the given @p map . Must be created if it does not exist.
  ///   static Voxel voxel(MAP &map, const Key &key, MapCache *cache);
  ///   /// Update the given @p voxel probability with a hit result.
  ///   static void integrateHit(MAP &map, Voxel &voxel, const glm::dvec3 & start, const glm::dvec3 &end);
  ///   /// Update the given @p voxel probability with a miss result.
  ///   static void integrateMiss(MAP &map, Voxel &voxel, const glm::dvec3 & start, const glm::dvec3 &end);
  // };
  /// ```
  template <typename MAP, typename MAPINTERFACE = RayMapperInterface<MAP>>
  class RayMapperCpu : public RayMapper
  {
  public:
    /// Alias the @c MAPINTERFACE
    using MapInterface = MAPINTERFACE;

    /// Constructor, wrapping the interface around the given @p map .
    ///
    /// @param map The target map. Must outlive this class.
    RayMapperCpu(MAP *map)
      : map_(map)
    {}

    /// Performs the ray integration.
    ///
    /// This is updated in a single threaded fashion. For each ray we calculate the affected voxel @c Key set and
    /// update those voxels. Voxels along each line segment have their occupancy probabilty diminished, while
    /// the end voxel of each segment has the probability increase. This behaviour may be modified by the @p RayFlag
    /// bits in @p ray_update_flags .
    ///
    /// In addition to updating the voxel occupancy, a sample mean and covariance value may be maintain for each voxel
    /// depending on the map configuration.
    ///
    /// @param rays The array of start/end point pairs to integrate.
    /// @param element_count The number of @c glm::dvec3 elements in @p rays, which is twice the ray count.
    /// @param ray_update_flags @c RayFlag bitset used to modify the behaviour of this function. All flags are
    /// implemeted.
    size_t integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned ray_update_flags = kRfDefault) override;

  private:
    MAP *map_;  ///< Target map.
  };


  template <typename MAP, typename MAPINTERFACE>
  size_t RayMapperCpu<MAP, MAPINTERFACE>::integrateRays(const glm::dvec3 *rays, size_t element_count,
                                                        unsigned ray_update_flags)
  {
    KeyList keys;
    MapCache cache;
    bool clipped_sample_voxel;

    const OccupancyMap &occupancy_map = MapInterface::occupancyMap(*map_);
    const float occupancy_threshold_value = occupancy_map.occupancyThresholdValue();
    const RayFilterFunction ray_filter = MapInterface::rayFilter(*map_);
    const bool use_filter = bool(ray_filter);

    glm::dvec3 start, end;
    unsigned filter_flags;
    for (size_t i = 0; i < element_count; i += 2)
    {
      filter_flags = 0;
      start = rays[i];
      end = rays[i + 1];

      if (use_filter)
      {
        if (!ray_filter(&start, &end, &filter_flags))
        {
          // Bad ray.
          continue;
        }
      }

      clipped_sample_voxel = (filter_flags & kRffClippedEnd);

      if (!(ray_update_flags & kRfExcludeRay))
      {
        // Calculate line key for the last voxel if the end point has been clipped
        calculateSegmentKeys(keys, occupancy_map, start, end, clipped_sample_voxel);

        for (auto &&key : keys)
        {
          Voxel voxel = MapInterface::voxel(*map_, key, &cache);
          const float voxel_value = voxel.value();

          bool stop_traversal = false;
          if ((ray_update_flags & kRfStopOnFirstOccupied) && voxel_value >= occupancy_threshold_value &&
              voxel_value != voxel::invalidMarkerValue())
          {
            // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal after
            // update.
            stop_traversal = true;
          }

          // kRfClearOnly flag set => only affect occupied 7s.
          if (!(ray_update_flags & kRfClearOnly) || voxel_value >= occupancy_threshold_value)
          {
            MapInterface::integrateMiss(*map_, voxel, start, end);
          }

          if (stop_traversal)
          {
            // Found first occupied voxel and request is to stop on the first occupied voxel. Abort traversal.
            // Make sure we do not update the en voxel.
            clipped_sample_voxel = true;
            break;
          }
        }
      }

      if (!clipped_sample_voxel && !(ray_update_flags & kRfExcludeSample))
      {
        Voxel voxel = MapInterface::voxel(*map_, MapInterface::voxelKey(*map_, rays[i + 1]), &cache);
        if (!(ray_update_flags & kRfEndPointAsFree))
        {
          MapInterface::integrateHit(*map_, voxel, start, end);
        }
        else
        {
          MapInterface::integrateMiss(*map_, voxel, start, end);
        }
      }
    }

    return element_count / 2;
  }
}  // namespace ohm


#endif  // RAYMAPPER_H
