// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapDetail.h"

#include <ohm/MapInfo.h>
#include <ohm/OccupancyMap.h>

#include <array>

namespace ohm
{
HeightmapDetail::~HeightmapDetail() = default;

const glm::dvec3 &HeightmapDetail::upAxisNormal(UpAxis axis_id)
{
  static const std::array<glm::dvec3, 7> axes =  //
    {
      glm::dvec3(0, 0, -1),  // -Z
      glm::dvec3(0, -1, 0),  // -Y
      glm::dvec3(-1, 0, 0),  // -X
      glm::dvec3(1, 0, 0),   // X
      glm::dvec3(0, 1, 0),   // Y
      glm::dvec3(0, 0, 1),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int lookup_index = int(axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(axes.size()))
  {
    // Reference the dummy index.
    lookup_index = int(axes.size() - 1);
  };

  return axes[lookup_index];
}


int HeightmapDetail::surfaceIndexA(UpAxis up_axis_id)
{
  static const std::array<int, 7> indices =  //
    {
      0,  // -Z
      0,  // -Y
      1,  // -X
      1,  // X
      0,  // Y
      0,  // Z
      0,  // Dummy
    };

  int lookup_index = int(up_axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(indices.size()))
  {
    // Reference the dummy index.
    lookup_index = int(indices.size() - 1);
  };

  return indices[lookup_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalA(UpAxis axis_id)
{
  static const std::array<glm::dvec3, 7> axes =  //
    {
      glm::dvec3(1, 0, 0),   // -Z
      glm::dvec3(1, 0, 0),   // -Y
      glm::dvec3(0, -1, 0),  // -X
      glm::dvec3(0, 1, 0),   // X
      glm::dvec3(-1, 0, 0),  // Y
      glm::dvec3(1, 0, 0),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int lookup_index = int(axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(axes.size()))
  {
    // Reference the dummy index.
    lookup_index = int(axes.size() - 1);
  };

  return axes[lookup_index];
}


int HeightmapDetail::surfaceIndexB(UpAxis up_axis_id)
{
  static const std::array<int, 7> indices =  //
    {
      1,  // -Z
      2,  // -Y
      2,  // -X
      2,  // X
      2,  // Y
      1,  // Z
      1,  // Dummy
    };

  int lookup_index = int(up_axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(indices.size()))
  {
    // Reference the dummy index.
    lookup_index = int(indices.size() - 1);
  };

  return indices[lookup_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalB(UpAxis axis_id)
{
  static const std::array<glm::dvec3, 7> axes =  //
    {
      glm::dvec3(0, -1, 0),  // -Z
      glm::dvec3(0, 0, 1),   // -Y
      glm::dvec3(0, 0, 1),   // -X
      glm::dvec3(0, 0, 1),   // X
      glm::dvec3(0, 0, 1),   // Y
      glm::dvec3(0, 1, 0),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int lookup_index = int(axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(axes.size()))
  {
    // Reference the dummy index.
    lookup_index = int(axes.size() - 1);
  };

  return axes[lookup_index];
}


void HeightmapDetail::fromMapInfo(const MapInfo &info)
{
  up_axis_id = UpAxis(int(info.get("heightmap-axis")));
  ceiling = double(info.get("heightmap-ceiling"));
  min_clearance = double(info.get("heightmap-clearance"));
  floor = double(info.get("heightmap-floor"));
  ignore_voxel_mean = bool(info.get("heightmap-ignore-voxel-mean"));
  generate_virtual_surface = bool(info.get("heightmap-virtual-surface"));
  mode = HeightmapMode(int(info.get("heightmap-mode")));
  generate_virtual_surface = bool(info.get("heightmap-virtual-surface"));
  virtual_surface_filter_threshold = bool(info.get("heightmap-virtual-surface-filter-threshold"));
  promote_virtual_below = bool(info.get("heightmap-virtual-surface-promote"));
  updateAxis();
}


void HeightmapDetail::toMapInfo(MapInfo &info) const
{
  const std::string mode_name = heightmapModeToString(mode);
  info.set(MapValue("heightmap", true));
  info.set(MapValue("heightmap-axis-x", up.x));
  info.set(MapValue("heightmap-axis-y", up.y));
  info.set(MapValue("heightmap-axis-z", up.z));
  info.set(MapValue("heightmap-axis", int(up_axis_id)));
  info.set(MapValue("heightmap-ceiling", ceiling));
  info.set(MapValue("heightmap-clearance", min_clearance));
  info.set(MapValue("heightmap-floor", floor));
  info.set(MapValue("heightmap-ignore-voxel-mean", ignore_voxel_mean));
  info.set(MapValue("heightmap-mode", int(mode)));
  info.set(MapValue("heightmap-mode-name", mode_name.c_str()));
  info.set(MapValue("heightmap-virtual-surface", generate_virtual_surface));
  info.set(MapValue("heightmap-virtual-surface-filter-threshold", int(virtual_surface_filter_threshold)));
  info.set(MapValue("heightmap-virtual-surface-promote", promote_virtual_below));
}
}  // namespace ohm
