//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef NDTMAP_H
#define NDTMAP_H

#include "OhmConfig.h"

#include "NdtMode.h"

#include <glm/fwd.hpp>

namespace ohm
{
class OccupancyMap;
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
/// this ellipsoid. This improves standard occupancy map issues of erosion. This calculation can only be made where
/// sufficient samples have been collected, as determined by @c ndtSampleThreshold() , otherwise a fixed probably
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
  /// @param mode The NDT mapping mode. Using @c NdtMode::kNone is invalid in the context and behaviour is undefined.
  NdtMap(OccupancyMap *map, bool borrowed_map, NdtMode mode = NdtMode::kOccupancy);

  /// Destructor: destroys the @c map() if @c borrowedMap() is false.
  ~NdtMap();

  /// Access the underlying occupancy map.
  OccupancyMap &map();
  /// Access the underlying occupancy map.
  const OccupancyMap &map() const;

  /// Specified the NDT mode the map is to use.
  NdtMode mode() const;

  /// True if @p map() is a borrowed pointer, false if the NDT map takes ownership.
  bool borrowedMap() const;

  /// Sets the adaptation rate for intersected NDT voxels. The value must be in the range [0, 1] with larger values
  /// yielding stronger effects.
  /// @param rate The adaptation rate to set [0, 1]
  void setAdaptationRate(float rate);
  /// Query the adaptation rate, which affects how quickly NDT logic removes intersected voxels.
  /// @return The current adaptation rate.
  float adaptationRate() const;

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

  /// Set the occupancy threshold value at which the covariance matrix may be reinitialised.
  ///
  /// This maps to the @c reinitialise_threshold parameter of @c calculateHitWithCovariance() . See that function for
  /// details.
  /// @param threshold The probability threshold value. Must be < 0 or behaviour is undefined.
  void setReinitialiseCovarianceThreshold(float threshold);

  /// Get the occupancy threshold value at which the covariance matrix may be reinitialised.
  /// @return The reset probability threshold value.
  float reinitialiseCovarianceThreshold() const;

  /// Set the occupancy threshold value at which the covariance matrix may be reinitialised.
  ///
  /// This maps to the @c reinitialise_sample_count parameter of @c calculateHitWithCovariance() . See that function
  /// for details.
  /// @param count The requires point count.
  void setReinitialiseCovariancePointCount(unsigned count);

  /// Get the occupancy threshold value at which the covariance matrix may be reinitialised.
  /// @return The reset point count threshold.
  unsigned reinitialiseCovariancePointCount() const;

  /// Set the initial covariance of intensity.
  ///
  /// @param initial_intensity_covariance The covariance of intensity for a voxel, for initialisation on receipt of
  /// first point. Must be greater than zero. Note that it is covariance, not standard deviation.
  void setInitialIntensityCovariance(float initial_intensity_covariance);
  /// Read the intensity covariance upon initialisation.
  float initialIntensityCovariance() const;

  /// Enable detailed tracing via 3rd Eye Scene.
  /// @param trace True to enable tracing.
  void setTrace(bool trace);
  /// Is tracing enabled?
  bool trace() const;

  /// Calculate an NDT adaptation rate which matches the strength of a pure occupancy miss probability.
  ///
  /// The NDT adaptation rate is a scaling factor for adjusting the probability on a miss using the NDT algorithm.
  /// While it may be set independently to the standard occupancy miss adjustment, this can yield much stronger NDT
  /// clearing effects than standard occupancy. This function seeks to keep the adaptation rate in line with the
  /// strength of the standard miss probability. That is, to achieve the same clearing effect as pure occupancy when
  /// the NDT algorithm yields a intersection probability of 1.
  ///
  /// The @p scale may be used to increase or decrease the NDT clearing effect in proportion with the pure occupancy
  /// clearing effect. For example, setting the scale to 1 will make the NDT clearing effect as strong as pure
  /// occupancy on a direct intersection with an occupied voxel's ellipsoid as described by the covariance matrix. The
  /// default scaling is 2 as NDT clearing effect is already dynamic, with increased effect the more directlly a ray
  /// passes through the ellipsoid.
  ///
  /// The adaptation rate is calculated as follows:
  ///
  /// @code{.unparsed}
  ///   adaptation_rate = scale * (1 - 2 * miss_probability)
  /// @endcode
  ///
  /// The result is then clamped to the range [0, 1].
  ///
  /// @param miss_probability The probability adjustment made on a pure occupancy miss, ranging (0, 0.5)
  /// @param scale Optional scaling factor for the result.
  /// @return The calculated NDT adaptation rate/scale factor [0, 1].
  static inline constexpr float ndtAdaptationRateFromMissProbability(float miss_probability, float scale = 2.0f)
  {
    return std::max(0.0f, std::min(scale * (1.0f - 2.0f * miss_probability), 1.0f));
  }

private:
  /// Update members of the underlying @c OccupancyMap::mapInfo() to include NDT parameters.
  void updateMapInfo();

  /// Enable NDT for the given @p map. This enables voxel mean positioning and adds a voxel layer to store the
  /// voxel covariance matrix approximation.
  /// @param map The occupancy map to enable NDT for.
  /// @param ndt_mode Specifies the NDT mapping mode. Must not be @c NdtMode::kNone.
  /// @return The voxel layer index for the covariance matrix.
  static int enableNdt(OccupancyMap *map, NdtMode ndt_mode);

  NdtMapDetail *imp_;
};
}  // namespace ohm

#endif  // NDTMAP_H
