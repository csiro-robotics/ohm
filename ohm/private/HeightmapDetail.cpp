// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "HeightmapDetail.h"

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