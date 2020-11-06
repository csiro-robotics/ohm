// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapDetail.h"

#include "MapInfo.h"

#include <array>

namespace ohm
{
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
    lookup_index = axes.size() - 1;
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
    lookup_index = indices.size() - 1;
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
    lookup_index = axes.size() - 1;
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
    lookup_index = indices.size() - 1;
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
    lookup_index = axes.size() - 1;
  };

  return axes[lookup_index];
}


void HeightmapDetail::fromMapInfo(const MapInfo &info)
{
  up_axis_id = UpAxis(int(info.get("heightmap-axis")));
  min_clearance = double(info.get("heightmap-clearance"));
  ceiling = double(info.get("heightmap-ceiling"));
  negative_obstacle_radius = double(info.get("heightmap-negative-obstacle-radius"));
  generate_virtual_surface = bool(info.get("heightmap-virtual-surface"));
  updateAxis();
}


void HeightmapDetail::toMapInfo(MapInfo &info) const
{
  info.set(MapValue("heightmap", true));
  info.set(MapValue("heightmap-axis", int(up_axis_id)));
  info.set(MapValue("heightmap-axis-x", up.x));
  info.set(MapValue("heightmap-axis-y", up.y));
  info.set(MapValue("heightmap-axis-z", up.z));
  info.set(MapValue("heightmap-clearance", min_clearance));
  info.set(MapValue("heightmap-ceiling", ceiling));
  info.set(MapValue("heightmap-negative-obstacle-radius", negative_obstacle_radius));
  info.set(MapValue("heightmap-virtual-surface", generate_virtual_surface));
}
}  // namespace ohm
