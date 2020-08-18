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
  /// This technique adds a voxel layer which includes a packed covariance matrix (see @c CovarianceVoxel ) and ensures
  /// voxel mean positioning is enabled.
  ///
  /// This covariance matrix describes an ellipsoid within the voxel which approximates the collection of samples used
  /// to populate that voxel. This is used to refine the probably adjustment of rays passing through the voxel.
  /// Conceptually, we adjust the probably according to the likelihood that the ray passing through the voxel intersects
  /// this ellipsoid. This improves standard occupancy map issues of errosion. This calculation can only be made where
  /// sufficient samples have been collected, as determined by @c ndtSampleThreshold() , otherwise a fxied probably
  /// adjustment is made as set by the @c OccupancyMap::missProbability() . This probably is also used as the learning
  /// rate for the NDT model.
  ///
  /// See paper:
  /// > 3D normal distributions transform occupancy maps: An efficient representation for mapping in dynamic
  /// > environments
  /// > Jari P. Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal
  class ohm_API NdtMap
  {
  public:
    /// Create an NDT map for the given @c OccupancyMap . The voxel layout is adjusted to include a layer for
    /// @c CovarianceVoxel data.
    ///
    /// The @p map ownership is passed to this object if @c borrowed_map is false, in which case the @c NdtMap takes
    /// ownership of the @c OccupancyMap .
    ///
    /// @param map The occupancy map to apply NDT updates to.
    /// @param borrowed_map True if @p map is a borrowed pointer. False if the NDT map takes ownership of the memory.
    NdtMap(OccupancyMap *map, bool borrowed_map = true);

    /// Destructor: destroys the @c map() if @c borrowedMap() is false.
    ~NdtMap();

    /// Access the underlying occupancy map.
    OccupancyMap &map();
    /// Access the underlying occupancy map.
    const OccupancyMap &map() const;

    /// True if @p map() is a borrowed pointer, false if the NDT map takes ownership.
    bool borrowedMap() const;

    /// Set the range sensor noise estimate. For example, the range noise for a lidar sensor.
    ///
    /// @param noise_range The sensor noise range. Must be greater than zero.
    void setSensorNoise(float noise_range);
    /// Read the range sensor noise estimate.
    float sensorNoise() const;

    /// Set the number of samples required in a voxel before using the NDT algorithm for @c integateMiss() adjustments.
    void setNdtSampleThreshold(unsigned sample_count);
    /// Get the number of samples required in a voxel before using the NDT algorithm for @c integateMiss() adjustments.
    unsigned ndtSampleThreshold();

    /// Set the occupancy theshold value at which the covariance matrix may be reinitialised.
    ///
    /// This maps to the @c reinitialise_threshold parameter of @c calculateHitWithCovariance() . See that function for
    /// details.
    /// @param theshold The probability theshold value. Must be < 0 or behaviour is undefined.
    void setReinitialiseCovarianceTheshold(float threshold);

    /// Get the occupancy theshold value at which the covariance matrix may be reinitialised.
    /// @return The reset probability theshold value.
    float reinitialiseCovarianceTheshold() const;

    /// Set the occupancy theshold value at which the covariance matrix may be reinitialised.
    ///
    /// This maps to the @c reinitialise_sample_count parameter of @c calculateHitWithCovariance() . See that function
    /// for details.
    /// @param count The requires point count.
    void setReinitialiseCovariancePointCount(unsigned count);

    /// Get the occupancy theshold value at which the covariance matrix may be reinitialised.
    /// @return The reset point count treshold.
    unsigned reinitialiseCovariancePointCount() const;

    /// Enable details tracing via 3rd Eye Scene.
    /// @param trace True to enable tracing.
    void setTrace(bool trace);
    /// Is tracing enabled?
    bool trace() const;

    /// Integrate a hit on @p voxel. Updates the voxel covariance.
    /// @param voxel The voxel containing the @p sample.
    /// @param sensor The location of the sensor from where the @p sample was detected.
    /// @param sample The sample location within the @p voxel.
    void integrateHit(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample);

    /// Integrate a miss on @p voxel. Uses the voxel covariance, but does not modify it.
    /// @param voxel A voxel the @p sensor to @p sample ray passes through..
    /// @param sensor The location of the sensor from where the @p sample was detected.
    /// @param sample The sample location.
    void integrateMiss(Voxel &voxel, const glm::dvec3 &sensor, const glm::dvec3 &sample);

    /// Debug render the NDT map ellipsoids via 3rd Eye Scene.
    void debugDraw() const;

  private:
    /// Update members of the underlying @c OccupancyMap::mapInfo() to include NDT parameters.
    void updateMapInfo();

    /// Enable NDT for the given @p map. This enables voxel mean positioning and adds a voxel layer to store the
    /// voxel covariance matrix approximation.
    /// @param map The occupancy map to enable NDT for.
    /// @return The voxel layer index for the covariance matrix.
    static int enableNdt(OccupancyMap *map);

    NdtMapDetail *imp_;
  };
}  // namespace ohm

#endif  // NDTMAP_H
