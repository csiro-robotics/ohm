// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RayPatternConical.h"

#include <3esservermacros.h>

#include <glm/ext.hpp>

#define DEBUG_VISUALISE 0

#if defined(TES_ENABLE) && !DEBUG_VISUALISE
#undef TES_ENABLE
#endif  // defined(TES_ENABLE) && !DEBUG_VISUALISE

using namespace ohm;

RayPatternConical::RayPatternConical(const glm::dvec3 &cone_axis, double cone_angle, double range,
                                     double angular_resolution, double min_range)
{
  TES_STMT(const double cone_radius = range * std::tan(cone_angle));
  TES_CONE_T(g_tes, TES_COLOUR_A(YellowGreen, 128), tes::Id(this),
             tes::Directional(tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(cone_axis)), cone_radius, range));
  TES_CONE_W(g_tes, TES_COLOUR_A(YellowGreen, 128), tes::Id(this) + 100,
             tes::Directional(tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(cone_axis)), cone_radius, range));

  // First ensure the cone axis is normalised.
  const glm::dvec3 &cone_normal = glm::normalize(cone_axis);

  TES_ARROW(g_tes, TES_COLOUR(Yellow), tes::Id(this),
            tes::Directional(tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(cone_normal))));

  // Add the cone axis.
  addRay(cone_normal * min_range, cone_normal * range);

  // Find a perpendicular to the cone normal. We will use this to define a radius vector around the unit circle through
  // the cone.
  // To generate, we simply sizzle the cone normal components.
  const glm::dvec3 deflection_base(cone_normal.z, cone_normal.x, cone_normal.y);

  TES_ARROW(g_tes, TES_COLOUR(Green), tes::Id(this) + 1,
            tes::Directional(tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(deflection_base))));

  TES_SERVER_UPDATE(g_tes, 0.0f);

  // Now walk inscribe the circle which makes the base.
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
  for (double circle_angle = 0; circle_angle < 2 * M_PI; circle_angle += angular_resolution)
  {
    // Define a deflection angle for this circle angle.
    const glm::dquat deflection_rotation = glm::angleAxis(circle_angle, cone_normal);
    // Define the deflection vector which we'll use to deflect the cone axis.
    const glm::dvec3 deflection_axis = deflection_rotation * deflection_base;

    TES_ARROW(g_tes, TES_COLOUR(Cyan), tes::Id(),
              tes::Directional(tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(deflection_axis))));

    // Now create deflected rates starting at angular_resolution up to the cone angle.
    // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
    for (double deflection_angle = angular_resolution; deflection_angle <= 0.5 * cone_angle;
         deflection_angle += angular_resolution)
    {
      // Rotate the cone_normal by the deflection angle around the deflection axis and scale by range.
      const glm::dquat rotation = glm::angleAxis(deflection_angle, deflection_axis);
      const glm::dvec3 ray_dir = rotation * cone_normal;
      addRay(ray_dir * min_range, ray_dir * range);

      TES_LINE(g_tes, TES_COLOUR(PowderBlue), tes::Id(), tes::Vector3d(0.0), tes::Vector3d(glm::value_ptr(ray_dir)));
    }

    TES_SERVER_UPDATE(g_tes, 0.0f);
  }

#ifdef TES_ENABLE
  std::vector<glm::vec3> debug_points;
  debug_points.reserve(2 * pointCount());
  const glm::dvec3 *points = this->points();
  for (size_t i = 0; i < pointCount(); ++i)
  {
    debug_points.push_back(glm::vec3(0.0f));
    debug_points.push_back(glm::vec3(points[i]));
  }
  TES_IF(!debug_points.empty())
  {
    TES_LINES(g_tes, TES_COLOUR(SteelBlue), glm::value_ptr(debug_points.front()), debug_points.size(),
              sizeof(debug_points.front()));
  }
#endif  // TES_ENABLE

  TES_SERVER_UPDATE(g_tes, 0.0f);
  TES_ARROW_END(g_tes, tes::Id(this));
  TES_ARROW_END(g_tes, tes::Id(this) + 1);
  TES_CONE_END(g_tes, tes::Id(this));
  TES_CONE_END(g_tes, tes::Id(this) + 100);
}
