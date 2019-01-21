// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapDetail.h"

#include "MapInfo.h"

using namespace ohm;

const glm::dvec3 &HeightmapDetail::upAxisNormal(int axis_id)
{
  static const glm::dvec3 kAxes[] =  //
    {
      glm::dvec3(0, 0, -1),  // -Z
      glm::dvec3(0, -1, 0),  // -Y
      glm::dvec3(-1, 0, 0),  // -X
      glm::dvec3(1, 0, 0),   // X
      glm::dvec3(0, 1, 0),   // Y
      glm::dvec3(0, 0, 1),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int axis_index = int(axis_id + 3);
  if (axis_index < 0 || axis_index >= int(sizeof(kAxes) - sizeof(kAxes[0])))
  {
    // Reference the dummy index.
    axis_index = int(sizeof(kAxes) - sizeof(kAxes[0]) - 1);
  };

  return kAxes[axis_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalA(int axis_id)
{
  static const glm::dvec3 kAxes[] =  //
    {
      glm::dvec3(1, 0, 0),   // -Z
      glm::dvec3(1, 0, 0),   // -Y
      glm::dvec3(0, -1, 0),  // -X
      glm::dvec3(0, 1, 0),   // X
      glm::dvec3(-1, 0, 0),  // Y
      glm::dvec3(1, 0, 0),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int axis_index = int(axis_id + 3);
  if (axis_index < 0 || axis_index >= int(sizeof(kAxes) - sizeof(kAxes[0])))
  {
    // Reference the dummy index.
    axis_index = int(sizeof(kAxes) - sizeof(kAxes[0]) - 1);
  };

  return kAxes[axis_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalB(int axis_id)
{
  static const glm::dvec3 kAxes[] =  //
    {
      glm::dvec3(0, -1, 0),  // -Z
      glm::dvec3(0, 0, 1),   // -Y
      glm::dvec3(0, 0, 1),   // -X
      glm::dvec3(0, 0, 1),   // X
      glm::dvec3(0, 0, 1),   // Y
      glm::dvec3(0, 1, 0),   // Z
      glm::dvec3(0, 0, 0),   // Dummy
    };

  int axis_index = int(axis_id + 3);
  if (axis_index < 0 || axis_index >= int(sizeof(kAxes) - sizeof(kAxes[0])))
  {
    // Reference the dummy index.
    axis_index = int(sizeof(kAxes) - sizeof(kAxes[0]) - 1);
  };

  return kAxes[axis_index];
}


void HeightmapDetail::fromMapInfo(const MapInfo &info)
{
  up_axis_id = int(info.get("heightmap-axis"));
  blur_level = int(info.get("heightmap-blur"));
  min_clearance = float(info.get("heightmap-clearance"));
  floor = float(info.get("heightmap-floor"));
  ceiling = float(info.get("heightmap-ceiling"));
  updateAxis();
}


void HeightmapDetail::toMapInfo(MapInfo &info) const
{
  info.set(MapValue("heightmap", true));
  info.set(MapValue("heightmap-axis", int(up_axis_id)));
  info.set(MapValue("heightmap-axis-x", up.x));
  info.set(MapValue("heightmap-axis-y", up.y));
  info.set(MapValue("heightmap-axis-z", up.z));
  info.set(MapValue("heightmap-blur", blur_level));
  info.set(MapValue("heightmap-clearance", min_clearance));
  info.set(MapValue("heightmap-floor", floor));
  info.set(MapValue("heightmap-ceiling", ceiling));
}
