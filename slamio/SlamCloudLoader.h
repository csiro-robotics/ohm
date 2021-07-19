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
/// This class is a means to an end and is not well implemented or maintained. There are three implementations for this
/// class, selected at build time based on available libraries:
/// - using liblas supporting reading LAS files with LAZ support if laszip is also available.
/// - using PDAL version 1.6 which supports PDAL available cloud formats without streaming
/// - using PDAL version 1.7+ which adds streaming support
///
/// The loader combines an input point cloud with a trajectory file where the trajectory file identifies the sensor
/// trajectory. This is used to match sample and sensor pairs. The trajectory file may be a point cloud with timestamps
/// or a whitespace delimited text file in the form `time x y z rot_x rot_y rot_z rot_w`. The first line of the text
/// file will be skipped on failure to parse under the assumption that it lists the column headings.
class slamio_API SlamCloudLoader
{
public:
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

  /// Attempt to preload the given number of points. Use zero to preload all. This does nothing with PDAL 1.6 as
  /// streaming is not supported in that version.
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
