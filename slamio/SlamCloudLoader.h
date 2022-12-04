// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_SLAMCLOUDLOADER_H_
#define SLAMIO_SLAMCLOUDLOADER_H_

#include "SlamIOExport.h"

#include "Points.h"

#include <functional>
#include <memory>

namespace slamio
{
struct SlamCloudLoaderDetail;

/// A utility class for loading a point cloud with a trajectory.
///
/// This class provides support for loading point cloud based sample files and trajectories. Minimum supported formats
/// are PLY point cloud loading (via miniply) and PLY or text file trajectory loading (see @c PointCloudReaderTraj ).
/// Building with @c OHM_FEATURE_PDAL enables PDAL support for other point cloud data types, with PLY loading unchanged.
/// PDAL version 1.7+ supports streaming loading.
///
/// The loader may be opened in one of three ways:
/// -# @c openWithTrajectory() to load separate point cloud and trajectories.
/// -# @c openPointCloud() to load data samples with no trajectory.
/// -# @c openRayCloud() to open a self contained ray cloud.
///
/// Using @c openWithTrajetory() , the loader combines an input point cloud with a trajectory file where the trajectory
/// file identifies the sensor trajectory. The timestamps in each file are used to used to match sample and sensor
/// pairs, thus both files must contain correlated timestamps. The @c SamplePoint::origin values are reported as
/// @c (0,0,0) when timestamps cannot be matched.
///
/// Using @c openPointCloud() opens a point cloud without trajectory. The @c SamplePoint::origin values are reported at
/// the same location as the @c SamplePoint::sample positions.
///
/// Using @c openRayCloud() opens only a point cloud which must have per point normal channels available. The normals
/// are treated as a non-normalised vector pointing from the sample point back to the sensor location at the time of
/// sampling. That is, `SamplePoint::origin = SamplePoint::sample + normal`. See
/// [RayCloudTools](https://github.com/csiro-robotics/raycloudtools) for more on ray clouds.
///
/// Typical usage:
///
/// @code
/// void readSlamCloud(const char *sample_file, const char *trajectory_file, std::vector<slamio::SamplePoint> &samples)
/// {
///   slamio::SlamCloudLoader reader;
///   // (optional) set error logging hook.
///   reader.setErrorLog([](const char *msg) { std::cerr << msg << std::flush; });
///   if (!reader.openWithTrajectory(sample_file, trajectory_file))
///   {
///     return;
///   }
///
///   // Note: numberOfPoints() may be unavailable (zero).
///   samples.reserve(reader.numberOfPoints());
///
///   slamio::SamplePoint sample{};
///   while (reader.nextSample(sample))
///   {
///     samples.emplace_back(sample);
///   }
/// }
/// @endcode
class slamio_API SlamCloudLoader
{
public:
  /// Logging function hook.
  using Log = std::function<void(const char *)>;

  /// Create a SLAM cloud loader.
  /// @param real_time_mode True to throttle point loading to simulate real time data acquisition.
  explicit SlamCloudLoader(bool real_time_mode = false);

  /// Set the error logging function.
  /// @param error_log Error loging function. May be empty to clear.
  void setErrorLog(Log error_log);
  /// Get the error logging function.
  Log errorLog() const;

  /// Destructor.
  ~SlamCloudLoader();

  /// Set the fixed offset between the trajectory point to the sensor frame. This is added to all trajectory points.
  /// @param offset The trajectory to sensor offset.
  void setSensorOffset(const glm::dvec3 &offset);

  /// Get the fixed offset between the trajectory point to the sensor frame.
  glm::dvec3 sensorOffset() const;

  /// Enable or disable infering the return number. The return number may be inferred when the two sequential samples
  /// have precisely the same timestamp. This assumption is only valid for some data sets. This inference is only
  /// performed when there is no explicit @c return_number point attribute.
  ///
  /// Must be enabled before calling @c open() as this has no effect on an open data stream.
  ///
  /// @param enable True to allow infering the return number.
  void enableReturnNumberInference(bool enable);

  /// Check if return number inference is enabled. See @c enableReturnNumberInference() .
  bool returnNumberInference() const;

  /// Open the given point cloud and trajectory file pair. Both file must be valid. The @p sample_file_path must be a
  /// point cloud file, while @p trajectory_file_path can be either a point cloud file or a text trajectory.
  ///
  /// A text trajectory file contains:
  /// - A headings line (optional)
  /// - One sample per line formatted: `time x y z [additional_fields]`
  ///
  /// All values are floating point values (double precision supported) and whitespace separated. Any
  /// @c additional_fields are ignored.
  ///
  /// @c SamplePoint values are generated from the @p sample_file_path point cloud and timestamps are correlated against
  /// the @p trajectory_file_path to interpolate a sensor position at that time. The @c sensorOffset() is added before
  /// reporting the combined @c CloudSample via @c nextSample()
  ///
  /// @param sample_file_path Point cloud file name.
  /// @param trajectory_file_path Point cloud or trajectory file name.
  /// @return True on successfully opening both files.
  bool openWithTrajectory(const char *sample_file_path, const char *trajectory_file_path);

  /// Open the given point cloud file. This generates @c CloudSample values which have a fixed, zero @p origin value.
  /// @param sample_file_path Point cloud file name.
  /// @return True on successfully opening the point cloud.
  bool openPointCloud(const char *sample_file_path);

  /// Open the given ray cloud file. A ray cloud is a point cloud file where the normals channel is used to represent
  /// a vector from the position back to the ray origin.
  ///
  /// For more information on ray clouds see:
  /// - [RayCloudTools source repository](https://github.com/csiro-robotics/raycloudtools)
  /// - [RayCloudTools paper](https://ieeexplore.ieee.org/abstract/document/9444433)
  ///
  /// @param sample_file_path Ray cloud file name.
  /// @return True on successfully opening the ray cloud.
  bool openRayCloud(const char *sample_file_path);

  /// Close the current input files.
  void close();

  /// Query the number of points. May be zero as some readers do not report the total point count.
  size_t numberOfPoints() const;

  /// Running in real time mode with points given at a rate determined by the the point cloud timestamps?
  bool realTimeMode() const;

  /// Do we have a point cloud?
  bool sampleFileIsOpen() const;
  /// Do we have a trajectory?
  bool trajectoryFileIsOpen() const;

  /// True if the input data has a timestamp channel.
  bool hasTimestamp() const;

  /// True if the input data has origin points for the sensor samples.
  bool hasOrigin() const;

  /// True if the input data has an intensity channel.
  bool hasIntensity() const;

  /// True if the input data has colour channels.
  bool hasColour() const;

  /// True if the input data has lidar point return numbers.
  bool hasReturnNumber() const;

  /// Attempt to preload the given number of points. Use zero to preload all. This does nothing with PDAL 1.6 as
  /// streaming is not supported in that version. PDAL 1.8 allow streaming.
  void preload(size_t point_count = 0);

  /// Get the next point, sensor position and timestamp.
  bool nextSample(SamplePoint &sample);

private:
  bool open(const char *sample_file_path, const char *trajectory_file_path, bool ray_cloud);

  bool loadPoint();

  /// Sample the trajectory at the given timestamp.
  ///
  /// This reads the trajectory to the segment which covers @p timestamp and
  /// linearly interpolates a @p position at this time.
  ///
  /// @param[out] position Set to the trajectory position on success.
  /// @param timestamp The desired sample time.
  /// @return True on success, false when @p timestamp is out of range.
  bool sampleTrajectory(glm::dvec3 &position, const glm::dvec3 &sample, double timestamp);

  std::unique_ptr<SlamCloudLoaderDetail> imp_;
};
}  // namespace slamio

#endif  // SLAMIO_SLAMCLOUDLOADER_H_
