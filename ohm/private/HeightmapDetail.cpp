// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapDetail.h"

#include "MapInfo.h"

using namespace ohm;

const glm::dvec3 &HeightmapDetail::upAxisNormal(UpAxis axis_id)
{
  static const glm::dvec3 axes[] =  //
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
  if (lookup_index < 0 || lookup_index >= int(sizeof(axes) - sizeof(axes[0])))
  {
    // Reference the dummy index.
    lookup_index = int(sizeof(axes) / sizeof(axes[0]) - 1);
  };

  return axes[lookup_index];
}


int HeightmapDetail::surfaceIndexA(UpAxis up_axis_id)
{
  static const int indices[] =  //
    {
      0, // -Z
      0, // -Y
      1, // -X
      1, // X
      0, // Y
      0, // Z
      0, // Dummy
    };

  int lookup_index = int(up_axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(sizeof(indices) - sizeof(indices[0])))
  {
    // Reference the dummy index.
    lookup_index = int(sizeof(indices) / sizeof(indices[0]) - 1);
  };

  return indices[lookup_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalA(UpAxis axis_id)
{
  static const glm::dvec3 axes[] =  //
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
  if (lookup_index < 0 || lookup_index >= int(sizeof(axes) - sizeof(axes[0])))
  {
    // Reference the dummy index.
    lookup_index = int(sizeof(axes) / sizeof(axes[0]) - 1);
  };

  return axes[lookup_index];
}


int HeightmapDetail::surfaceIndexB(UpAxis up_axis_id)
{
  static const int indices[] =  //
    {
      1, // -Z
      2, // -Y
      2, // -X
      2, // X
      2, // Y
      1, // Z
      1, // Dummy
    };

  int lookup_index = int(up_axis_id) + 3;
  if (lookup_index < 0 || lookup_index >= int(sizeof(indices) - sizeof(indices[0])))
  {
    // Reference the dummy index.
    lookup_index = int(sizeof(indices) / sizeof(indices[0]) - 1);
  };

  return indices[lookup_index];
}


const glm::dvec3 &HeightmapDetail::surfaceNormalB(UpAxis axis_id)
{
  static const glm::dvec3 axes[] =  //
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
  if (lookup_index < 0 || lookup_index >= int(sizeof(axes) - sizeof(axes[0])))
  {
    // Reference the dummy index.
    lookup_index = int(sizeof(axes) / sizeof(axes[0]) - 1);
  };

  return axes[lookup_index];
}


void HeightmapDetail::fromMapInfo(const MapInfo &info)
{
  up_axis_id = UpAxis(int(info.get("heightmap-axis")));
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
  info.set(MapValue("heightmap-clearance", min_clearance));
  info.set(MapValue("heightmap-floor", floor));
  info.set(MapValue("heightmap-ceiling", ceiling));
}
