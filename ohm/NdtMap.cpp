// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas, Jason Williams
#include "NdtMap.h"

#include "OccupancyMap.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "NdtVoxel.h"
#include "SubVoxel.h"
#include "Voxel.h"
#include "VoxelLayout.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/mat3x3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include <3esservermacros.h>
#ifdef TES_ENABLE
#include <3esserver.h>
#include <shapes/3esshapes.h>
#endif  // TES_ENABLE

#include <cassert>

using namespace ohm;

namespace ohm
{
  struct NdtMapDetail
  {
    OccupancyMap *map = nullptr;
    double sensor_noise = 0.05;
    int covariance_layer_index = -1;
    unsigned sample_threshold = 10;
    bool borrowed_map = false;
    bool trace = false;
  };
}  // namespace ohm


NdtMap::NdtMap(OccupancyMap *map, bool borrowed_map)
  : imp_(new NdtMapDetail)
{
  imp_->map = map;
  imp_->borrowed_map = borrowed_map;
  imp_->covariance_layer_index = enableNdt(map);
}


NdtMap::~NdtMap()
{
  if (imp_)
  {
    if (!imp_->borrowed_map)
    {
      delete imp_->map;
    }
    delete imp_;
  }
}


OccupancyMap &NdtMap::map()
{
  return *imp_->map;
}


const OccupancyMap &NdtMap::map() const
{
  return *imp_->map;
}


bool NdtMap::borrowedMap() const
{
  return imp_->borrowed_map;
}


void NdtMap::setSensorNoise(double noise_range)
{
  imp_->sensor_noise = noise_range;
}


double NdtMap::sensorNoise() const
{
  return imp_->sensor_noise;
}


void NdtMap::setNdtSampleThreshold(unsigned sample_count)
{
  imp_->sample_threshold = sample_count;
}


unsigned NdtMap::ndtSampleThreshold()
{
  return imp_->sample_threshold;
}


void NdtMap::setTrace(bool trace)
{
  imp_->trace = trace;
}


bool NdtMap::trace() const
{
  return imp_->trace;
}


int NdtMap::covarianceLayerIndex() const
{
  return imp_->covariance_layer_index;
}


void NdtMap::integrateHit(Voxel &voxel, const glm::dvec3 & /*sensor*/, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;

  assert(map.layout().hasSubVoxelPattern());
  OccupancyVoxel *voxel_occupancy = voxel.layerContent<OccupancyVoxel *>(map.layout().occupancyLayer());
  NdtVoxel *ndt_voxel = voxel.layerContent<NdtVoxel *>(imp_->covariance_layer_index);

  const glm::dvec3 voxel_centre = voxel.centreGlobal();
  glm::dvec3 voxel_mean;
#if OHM_NDT_UNPACKED_MEAN
  voxel_mean = glm::dvec3(ndt_voxel->mean[0], ndt_voxel->mean[1], ndt_voxel->mean[2]);
#else   // OHM_NDT_UNPACKED_MEAN
  voxel_mean = subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()) + voxel_centre;
#endif  // OHM_NDT_UNPACKED_MEAN

  float voxel_value = voxel.value();
  voxel_mean = calculateHit(ndt_voxel, &voxel_value, sample, voxel_mean, map.hitValue(),
                            map.occupancyThresholdValue(), voxel::invalidMarkerValue(), imp_->sensor_noise);

  // NDT probably value update is the same as for the basic occupancy map.
  voxel.setValue(voxel_value);

#if OHM_NDT_UNPACKED_MEAN
  ndt_voxel->mean[0] = voxel_mean[0];
  ndt_voxel->mean[1] = voxel_mean[1];
  ndt_voxel->mean[2] = voxel_mean[2];
#endif  //
  voxel_occupancy->sub_voxel = subVoxelCoord(voxel_mean - voxel_centre, map.resolution());
}


void NdtMap::integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample)
{
  OccupancyMap &map = *imp_->map;

  const float initial_value = voxel.value();
  float voxel_value = initial_value;
  NdtVoxel *ndt_voxel = voxel.layerContent<NdtVoxel *>(imp_->covariance_layer_index);

#if OHM_NDT_UNPACKED_MEAN
  const glm::dvec3 voxel_mean(ndt_voxel->mean[0], ndt_voxel->mean[1], ndt_voxel->mean[2]);
#else   // OHM_NDT_UNPACKED_MEAN
  assert(map.layout().hasSubVoxelPattern());
  const OccupancyVoxel *voxel_occupancy = voxel.layerContent<const OccupancyVoxel *>(map.layout().occupancyLayer());
  const glm::dvec3 voxel_mean(subVoxelToLocalCoord<glm::dvec3>(voxel_occupancy->sub_voxel, map.resolution()) +
                              voxel.centreGlobal());
#endif  // OHM_NDT_UNPACKED_MEAN

  const glm::dvec3 voxel_maximum_likelyhood =
    calculateMiss(ndt_voxel, &voxel_value, sensor, sample, voxel_mean, map.occupancyThresholdValue(),
                  voxel::invalidMarkerValue(), map.missValue(), imp_->sensor_noise, imp_->sample_threshold);
  voxel.setValue(voxel_value);

  TES_IF(imp_->trace)
  {
    TES_BOX_W(g_3es, TES_COLOUR(OrangeRed),
              TES_PTR_ID(voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index)),
              glm::value_ptr(voxel.centreGlobal()), glm::value_ptr(glm::dvec3(imp_->map->resolution())));
    TES_SERVER_UPDATE(g_3es, 0.0f);
  }

#ifdef TES_ENABLE
  TES_IF(imp_->trace)
  {
    bool drew_surfel = false;
    glm::dvec3 evals;
    glm::dmat3 evecs;
    if (eigenDecomposition(*ndt_voxel, &evals, &evecs))
    {
      glm::dquat rot(evecs);
      rot = glm::normalize(rot);
      TES_SPHERE(g_3es, TES_COLOUR(SeaGreen), TES_PTR_ID(ndt_voxel), glm::value_ptr(voxel_mean),
                 glm::value_ptr(2.0 * evals), tes::Quaterniond(rot.x, rot.y, rot.z, rot.w));
      drew_surfel = true;
    }

    // Trace the voxel mean, maximum likelyhood point and the ellipsoid.
    // Mean
    TES_SPHERE(g_3es, TES_COLOUR(OrangeRed), TES_PTR_ID(&voxel_mean), glm::value_ptr(voxel_mean), 0.05f);
    // Maximum likelyhood
    TES_SPHERE_W(g_3es, TES_COLOUR(PowderBlue), TES_PTR_ID(&voxel_maximum_likelyhood),
                 glm::value_ptr(voxel_maximum_likelyhood), 0.1f);

    glm::dvec3 pos = voxel.centreGlobal();
    char text[64];
    sprintf(text, "P %.3f", ohm::valueToProbability(voxel_value - initial_value));
    TES_TEXT2D_WORLD(g_3es, TES_COLOUR(White), text, glm::value_ptr(pos));

    TES_SERVER_UPDATE(g_3es, 0.0f);
    TES_BOX_END(g_3es, TES_PTR_ID(ndt_voxel));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_mean));
    TES_SPHERE_END(g_3es, TES_PTR_ID(&voxel_maximum_likelyhood));
    if (drew_surfel)
    {
      TES_SPHERE_END(g_3es, TES_PTR_ID(ndt_voxel));
    }
  }
#endif  // TES_ENABLE
}


int NdtMap::enableNdt(OccupancyMap *map)
{
  map->setSubVoxelsEnabled(true);

  // We need additional layers in the map to store information for the NDT update.
  MapLayout layout = map->layout();
  MapLayer &layer = *layout.addLayer("covariance");
  VoxelLayout voxel = layer.voxelLayout();
  // Add members to represent a diagonal of the covariance matrix. This is an approximation of the full matrix
  // but it greatly reduces the per voxel memory usage.
  voxel.addMember("P00", DataType::kFloat, 0);
  voxel.addMember("P11", DataType::kFloat, 0);
  voxel.addMember("P22", DataType::kFloat, 0);
  voxel.addMember("P33", DataType::kFloat, 0);
  voxel.addMember("P44", DataType::kFloat, 0);
  voxel.addMember("P55", DataType::kFloat, 0);
#if OHM_NDT_UNPACKED_MEAN
  voxel.addMember("mean_x", DataType::kFloat, 0);
  voxel.addMember("mean_y", DataType::kFloat, 0);
  voxel.addMember("mean_z", DataType::kFloat, 0);
#endif  // OHM_NDT_UNPACKED_MEAN
  voxel.addMember("sample_count", DataType::kUInt32, 0);

  map->updateLayout(layout);
  return layer.layerIndex();
}


void NdtMap::debugDraw()
{
#ifdef TES_ENABLE
  if (!g_3es || g_3es->connectionCount() == 0)
  {
    return;
  }

  std::vector<tes::Sphere> ellipsoids;
  std::vector<tes::Shape *> shape_ptrs;

  const auto send = [&ellipsoids, &shape_ptrs]()  //
  {
    if (!ellipsoids.empty())
    {
      shape_ptrs.clear();
      for (size_t i = 0; i < ellipsoids.size(); ++i)
      {
        shape_ptrs.emplace_back(&ellipsoids[i]);
      }

      g_3es->create(tes::MultiShape(shape_ptrs.data(), shape_ptrs.size()));
      ellipsoids.clear();
    }
  };

  uint32_t next_id = static_cast<uint32_t>((size_t)this);
  const tes::Colour c = tes::Colour::Colours[tes::Colour::SeaGreen];
  for (auto iter = imp_->map->begin(); iter != imp_->map->end(); ++iter)
  {
    const auto voxel = *iter;
    if (voxel.isOccupied())
    {
      const NdtVoxel &ndt_voxel = *voxel.layerContent<const NdtVoxel *>(imp_->covariance_layer_index);
      glm::dvec3 evals;
      glm::dmat3 evecs;
      if (!eigenDecomposition(ndt_voxel, &evals, &evecs))
      {
        continue;
      }

#if OHM_NDT_UNPACKED_MEAN
      const glm::dvec3 voxel_mean(ndt_voxel.mean[0], ndt_voxel.mean[1], ndt_voxel.mean[2]);
#else   // OHM_NDT_UNPACKED_MEAN
      const glm::dvec3 voxel_mean = voxel.position();
#endif  // OHM_NDT_UNPACKED_MEAN
      tes::Sphere ellipsoid(next_id, glm::value_ptr(voxel_mean));
      glm::dquat q(evecs);
      ellipsoid.setRotation(tes::Quaterniond(q.x, q.y, q.z, q.w));
      ellipsoid.setScale(2.0 * tes::Vector3d(evals[0], evals[1], evals[2]));
      ellipsoid.setColour(c);
      ellipsoids.emplace_back(ellipsoid);

      if (ellipsoids.size() >= tes::MultiShape::ShapeCountLimit)
      {
        send();
        ++next_id;
      }
    }
  }

  send();

  TES_SERVER_UPDATE(ohm::g_3es, 0.0f);
#endif  // TES_ENABLE
}
