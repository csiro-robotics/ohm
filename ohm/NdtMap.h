//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef NDTMAP_H
#define NDTMAP_H

#include "OhmConfig.h"

#include <glm/fwd.hpp>

namespace ohm
{
  class OccupancyMap;
  class Voxel;
  struct NdtMapDetail;

  /// A variation on @c OccupancyMap which uses a 3D Normal Distribution Transform (NDT) to calculate occupancy
  /// adjustments.
  ///
  /// See paper:
  /// > 3D normal distributions transform occupancy maps: An efficient representation for mapping in dynamic
  /// > environments
  /// > Jari P. Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal
  ///
  class ohm_API NdtMap
  {
  public:
    NdtMap(OccupancyMap *map, bool borrowed_map = true);

    ~NdtMap();

    OccupancyMap &map();
    const OccupancyMap &map() const;

    bool borrowedMap() const;

    void setSensorNoise(double noise_range);
    double sensorNoise() const;

    /// Set the number of samples required in a voxel before using the NDT algorithm for @c integateMiss() adjustments.
    void setNdtSampleThreshold(unsigned sample_count);
    unsigned ndtSampleThreshold();

    void setTrace(bool trace);
    bool trace() const;

    /// Query the voxel layer index which stores the @c NdtVoxel covariance data.
    /// @return The covariance voxel layer index.
    int covarianceLayerIndex() const;

    void integrateHit(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample);
    void integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample);

    void debugDraw();

  private:
    /// Enable NDT for the given @p map. This enables sub-voxel positioning and adds a voxel layer to store the
    /// voxel covariance matrix approximation.
    /// @param map The occupancy map to enable NDT for.
    /// @return The voxel layer index for the covariance matrix.
    static int enableNdt(OccupancyMap *map);

    NdtMapDetail *imp_;
  };
}  // namespace ohm

#endif  // NDTMAP_H
