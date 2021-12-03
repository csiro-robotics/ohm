// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "DebugDraw.h"

#include "OccupancyMap.h"
#include "Trace.h"
#include "VoxelData.h"

#include <3esservermacros.h>

#include <glm/gtc/type_ptr.hpp>

#ifdef TES_ENABLE
namespace
{
template <typename ShapeType>
void sendShapes(std::vector<ShapeType> &shapes, std::vector<tes::Shape *> &shape_ptrs)
{
  if (!shapes.empty() && ohm::g_tes)
  {
    shape_ptrs.clear();
    for (auto &ellipsoid : shapes)
    {
      shape_ptrs.emplace_back(&ellipsoid);
    }

    ohm::g_tes->create(tes::MultiShape(shape_ptrs.data(), shape_ptrs.size()));
    shapes.clear();
  }
}
}  // namespace
#endif  // TES_ENABLE

namespace ohm
{
bool debugDraw(const ohm::OccupancyMap &map)
{
#ifdef TES_ENABLE
  if (map.layout().intensityLayer() >= 0 && map.layout().hitMissCountLayer() >= 0)
  {
    return debugDrawNdtTm(map);
  }
  if (map.layout().covarianceLayer() >= 0)
  {
    return debugDrawNdtOm(map);
  }
  // Occupancy only
  return debugDrawOccupancy(map);
#else   // TES_ENABLE
  (void)map;
  return false;
#endif  // TES_ENABLE
}


bool debugDrawNdtTm(const ohm::OccupancyMap &map)
{
  if (!trace::available())
  {
    return false;
  }
#ifdef TES_ENABLE
  if (!g_tes || g_tes->connectionCount() == 0)
  {
    // No connections. Nothing to send data to, so do no work.
    return true;
  }

  // NDT-TM
  static float min_intensity = 0.0f, max_intensity = 255.0f;
  auto next_id = static_cast<uint32_t>(size_t(&map));
  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<const CovarianceVoxel> cov(&map, map.layout().covarianceLayer());
  Voxel<const VoxelMean> mean(&map, map.layout().meanLayer());
  Voxel<const IntensityMeanCov> intensity_voxel(&map, map.layout().intensityLayer());
  Voxel<const HitMissCount> hit_miss_voxel(&map, map.layout().hitMissCountLayer());

  if (!occupancy.isLayerValid() || !cov.isLayerValid() || !mean.isLayerValid() || !intensity_voxel.isLayerValid() ||
      !hit_miss_voxel.isLayerValid())
  {
    return false;
  }

  std::vector<tes::Sphere> ellipsoids;
  std::vector<tes::Shape *> shape_ptrs;
  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    occupancy.setKey(*iter);
    if (isOccupied(occupancy))
    {
      cov.setKey(occupancy);

      glm::dquat rot;
      glm::dvec3 scale;
      CovarianceVoxel cv;
      cov.read(&cv);
      if (!covarianceUnitSphereTransformation(&cv, &rot, &scale))
      {
        continue;
      }

      mean.setKey(occupancy);
      intensity_voxel.setKey(occupancy);
      hit_miss_voxel.setKey(occupancy);

      const glm::dvec3 voxel_mean = positionUnsafe(mean);

      IntensityMeanCov intensity_mean_cov;
      intensity_voxel.read(&intensity_mean_cov);
      min_intensity = std::fmin(min_intensity, intensity_mean_cov.intensity_mean);
      max_intensity = std::fmax(max_intensity, intensity_mean_cov.intensity_mean);
      const float scaled_intensity =
        float(M_PI) * (-0.75f + 1.5f * std::log(intensity_mean_cov.intensity_mean - min_intensity + 1.0f) /
                                  std::fmax(1.0f, std::log(max_intensity - min_intensity + 1.0f)));
      const float sin_sc = std::sin(scaled_intensity), cos_sc = std::cos(scaled_intensity);
      HitMissCount hit_miss_count;
      hit_miss_voxel.read(&hit_miss_count);

      tes::Sphere ellipsoid(next_id, tes::Spherical(tes::Vector3d(glm::value_ptr(voxel_mean))));
      ellipsoid.setRotation(tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      ellipsoid.setScale(2.0 * tes::Vector3d(scale.x, scale.y, scale.z));
      ellipsoid.setColour(
        tes::Colour(0.5f * (1.0f + sin_sc), 0.5f * (1.0f + cos_sc), 0.5f * (1.0f - sin_sc),
                    0.1f + 0.89f * float(hit_miss_count.hit_count) /
                             std::fmax(1.0f, float(hit_miss_count.hit_count + hit_miss_count.miss_count))));
      ellipsoid.setTransparent(true);

      // const float alpha = 0.9f * float(hit_miss_count.hit_count) /
      //                    std::fmax(1.0f, float(hit_miss_count.hit_count + hit_miss_count.miss_count));
      // ellipsoid.setColour(tes::Colour(0.1f + alpha * 0.5f * (1.0f + sin_sc), 0.1f + alpha * 0.5f * (1.0f + cos_sc),
      //                                0.1f + alpha * 0.5f * (1.0f - sin_sc), 1.0f));

      ellipsoids.emplace_back(ellipsoid);

      if (ellipsoids.size() >= tes::MultiShape::ShapeCountLimit)
      {
        sendShapes(ellipsoids, shape_ptrs);
        ++next_id;
      }
    }
  }
  sendShapes(ellipsoids, shape_ptrs);

  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
  return true;
#else   // TES_ENABLE
  (void)map;
  return false;
#endif  // TES_ENABLE
}


bool debugDrawNdtOm(const ohm::OccupancyMap &map)
{
  if (!trace::available())
  {
    return false;
  }
#ifdef TES_ENABLE
  if (!g_tes || g_tes->connectionCount() == 0)
  {
    // No connections. Nothing to send data to, so do no work.
    return true;
  }

  // NDT-OM
  std::vector<tes::Sphere> ellipsoids;
  std::vector<tes::Shape *> shape_ptrs;
  const tes::Colour c = tes::Colour::Colours[tes::Colour::SeaGreen];
  auto next_id = static_cast<uint32_t>(size_t(&map));
  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<const CovarianceVoxel> cov(&map, map.layout().covarianceLayer());
  Voxel<const VoxelMean> mean(&map, map.layout().meanLayer());

  if (!occupancy.isLayerValid() || !cov.isLayerValid() || !mean.isLayerValid())
  {
    return false;
  }

  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    occupancy.setKey(*iter);
    if (isOccupied(occupancy))
    {
      cov.setKey(occupancy);

      glm::dquat rot;
      glm::dvec3 scale;
      CovarianceVoxel cv;
      if (!cov.isValid())
      {
        // Should be impossible, but helps clang-tidy.
        continue;
      }
      cov.read(&cv);
      if (!covarianceUnitSphereTransformation(&cv, &rot, &scale))
      {
        continue;
      }

      mean.setKey(occupancy);

      const glm::dvec3 voxel_mean = positionUnsafe(mean);

      tes::Sphere ellipsoid(next_id, tes::Spherical(tes::Vector3d(glm::value_ptr(voxel_mean))));
      ellipsoid.setRotation(tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      ellipsoid.setScale(2.0 * tes::Vector3d(scale.x, scale.y, scale.z));
      ellipsoid.setColour(c);
      ellipsoids.emplace_back(ellipsoid);

      if (ellipsoids.size() >= tes::MultiShape::ShapeCountLimit)
      {
        sendShapes(ellipsoids, shape_ptrs);
        ++next_id;
      }
    }
  }
  sendShapes(ellipsoids, shape_ptrs);

  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
  return true;
#else   // TES_ENABLE
  (void)map;
  return false;
#endif  // TES_ENABLE
}


bool debugDrawOccupancy(const ohm::OccupancyMap &map)
{
  if (!trace::available())
  {
    return false;
  }
#ifdef TES_ENABLE
  if (!g_tes || g_tes->connectionCount() == 0)
  {
    // No connections. Nothing to send data to, so do no work.
    return true;
  }

  // occupancy
  std::vector<tes::Box> boxes;
  std::vector<tes::Shape *> shape_ptrs;
  auto next_id = static_cast<uint32_t>(size_t(&map));
  Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  Voxel<const VoxelMean> mean(&map, map.layout().meanLayer());
  float occupancy_value = 0;
  const tes::Vector3d scale(map.resolution());

  // Ok for mean layer to be invalid. We check that below.
  if (!occupancy.isLayerValid())
  {
    return false;
  }

  for (auto iter = map.begin(); iter != map.end(); ++iter)
  {
    occupancy.setKey(*iter);
    if (isOccupied(occupancy))
    {
      mean.setKey(occupancy);
      const glm::dvec3 voxel_mean = positionSafe(mean);

      occupancy_value = occupancy.data();
      occupancy_value = valueToProbability(occupancy_value);
      const auto intensity =
        float((occupancy_value - map.occupancyThresholdProbability()) / (1.0 - map.occupancyThresholdProbability()));
      const int c = int(255 * intensity);

      tes::Box box(next_id, tes::Transform(tes::Vector3d(glm::value_ptr(voxel_mean)), scale));
      box.setColour(tes::Colour(c, c, c));
      boxes.emplace_back(box);

      if (boxes.size() >= tes::MultiShape::ShapeCountLimit)
      {
        sendShapes(boxes, shape_ptrs);
        ++next_id;
      }
    }
  }
  sendShapes(boxes, shape_ptrs);

  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
  return true;
#else   // TES_ENABLE
  (void)map;
  return false;
#endif  // TES_ENABLE
}
}  // namespace ohm
