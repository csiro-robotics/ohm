//
// author Kazys Stepanas
//

#include "slamioexport.h"

#include <glm/glm.hpp>
#include <glm/ext.hpp>

struct SlamCloudLoaderDetail;

class slamio_API SlamCloudLoader
{
public:
  /// Create a SLAM cloud loader.
  /// @param realTimeMode True to throttle point loading to simulate real time data acquisition.
  SlamCloudLoader(bool realTimeMode = false);
  ~SlamCloudLoader();

  bool open(const char *sampleFilePath, const char *trajectoryFilePath);

  void close();

  size_t numberOfPoints() const;

  bool realTimeMode() const;

  bool sampleFileIsOpen() const;
  bool trajectoryFileIsOpen() const;

  void preload(size_t pointCount = 0);

  bool nextPoint(glm::dvec3 &sample, glm::dvec3 *origin = nullptr, double *timestamp = nullptr, glm::dquat *orientation = nullptr);

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
  bool sampleTrajectory(glm::dvec3 &position, glm::dquat &orientation, double timestamp);

  SlamCloudLoaderDetail *_imp;
};
