//
// author Kazys Stepanas
//

#include "SlamIOExport.h"

#include <glm/ext.hpp>
#include <glm/glm.hpp>

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
  /// Create a SLAM cloud loader.
  /// @param real_time_mode True to throttle point loading to simulate real time data acquisition.
  SlamCloudLoader(bool real_time_mode = false);
  ~SlamCloudLoader();

  /// Set the fixed offset between the trajectory point to the sensor frame. This is added to all trajectory points.
  /// @param offset The trajectory to sensor offset.
  void setSensorOffset(const glm::dvec3 &offset);
  /// Get the fixed offset between the trajectory point to the sensor frame.
  glm::dvec3 sensorOffset() const;

  /// Open the given point cloud and trajectory file pair. The trajectory file may be empty "" to omit.
  bool open(const char *sample_file_path, const char *trajectory_file_path);

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

  /// Attempt to preload the given number of points. Use zero to preload all. This does nothing with PDAL 1.6 as
  /// streaming is not supported in that version.
  void preload(size_t point_count = 0);

  /// Get the next point, sensor position and timestamp.
  bool nextPoint(glm::dvec3 &sample, glm::dvec3 *origin = nullptr, double *timestamp = nullptr);

private:
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

  SlamCloudLoaderDetail *imp_;
};
