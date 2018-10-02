//
// author Kazys Stepanas
//
#include "slamcloudloader.h"

#ifdef _MSC_VER
// std::equal with parameters that may be unsafe warning under Visual Studio.
#pragma warning(disable : 4996)
#endif // _MSC_VER

#include <ohmutil/safeio.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <chrono>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <liblas/liblas.hpp>
#include <liblas/factory.hpp>
#include <liblas/reader.hpp>

namespace
{
  struct TrajectoryPoint
  {
    double timestamp;
    glm::dvec3 origin;
    glm::dquat orientation;
  };

  struct SamplePoint : TrajectoryPoint
  {
    glm::dvec3 sample;
  };

  typedef std::chrono::high_resolution_clock clock;
}

struct SlamCloudLoaderDetail
{
  liblas::ReaderFactory *lasReaderFactory;
  liblas::Reader *sampleReader;
  liblas::Reader *trajectoryReader;
  std::string sampleFilePath;
  std::string trajectoryFilePath;
  std::ifstream sampleFile;
  std::ifstream trajectoryFile;
  std::function<bool(TrajectoryPoint &point)> readTrajectoryPoint;
  std::string trajLine;
  TrajectoryPoint trajectoryBuffer[2];
  size_t numberOfPoints;

  std::vector<SamplePoint> samplesBuffer;
  size_t nextSampleReadIndex;

  clock::time_point firstSampleReadTime;
  double firstSampleTimestamp;
  bool realTimeMode;

  inline SlamCloudLoaderDetail()
    : lasReaderFactory(nullptr)
    , sampleReader(nullptr)
    , trajectoryReader(nullptr)
    , numberOfPoints(0)
    , nextSampleReadIndex(0)
    , firstSampleTimestamp(-1.0)
    , realTimeMode(false)
  {
    memset(&trajectoryBuffer, 0, sizeof(trajectoryBuffer));
  }

  inline ~SlamCloudLoaderDetail()
  {
    // Factory must be destroyed last.
    delete trajectoryReader;
    delete sampleReader;
    delete lasReaderFactory;
  }
};

namespace
{
  std::string getFileExtension(const std::string &file)
  {
    size_t lastDot = file.find_last_of(".");
    if (lastDot != std::string::npos)
    {
      return file.substr(lastDot + 1);
    }

    return "";
  }

  bool openLasFile(std::ifstream &in, const std::string &fileName)
  {
    const std::string ext = getFileExtension(fileName);
    if (ext.compare("laz") == 0 || ext.compare("las") == 0)
    {
      return liblas::Open(in, fileName);
    }

    // Extension omitted.
    // Try for a LAZ file (compressed), then a LAS file.
    if (liblas::Open(in, fileName + ".laz"))
    {
      return true;
    }
    return liblas::Open(in, fileName + ".las");
  }
}

SlamCloudLoader::SlamCloudLoader(bool realTimeMode)
  : _imp(new SlamCloudLoaderDetail)
{
  _imp->lasReaderFactory = new liblas::ReaderFactory;
  _imp->realTimeMode = realTimeMode;
}


SlamCloudLoader::~SlamCloudLoader()
{
  delete _imp;
}


bool SlamCloudLoader::open(const char *sampleFilePath, const char *trajectoryFilePath)
{
  close();

  _imp->sampleFilePath = sampleFilePath;
  _imp->trajectoryFilePath = trajectoryFilePath;

  openLasFile(_imp->sampleFile, _imp->sampleFilePath);
  bool textTrajectory = false;
  {
    const std::string trajExt = getFileExtension(trajectoryFilePath);
    if (trajExt.compare("txt") == 0)
    {
      // Text based trajectory.
      _imp->trajectoryFile.open(trajectoryFilePath, std::ios::binary);
      textTrajectory = true;
    }
    else
    {
      openLasFile(_imp->trajectoryFile, _imp->trajectoryFilePath);
    }
  }

  if (!sampleFileIsOpen() || !trajectoryFileIsOpen())
  {
    close();
    return false;
  }

  _imp->sampleReader = new liblas::Reader(_imp->lasReaderFactory->CreateWithStream(_imp->sampleFile));

  if (textTrajectory)
  {
    _imp->readTrajectoryPoint = [this](TrajectoryPoint &point) -> bool
    {
      if (!std::getline(_imp->trajectoryFile, _imp->trajLine))
      {
        // End of file.
        return false;
      }

      // sscanf is far faster than using stream operators.
      if (sscanf_s(_imp->trajLine.c_str(), "%lg %lg %lg %lg %lg %lg %lg %lg",
                 &point.timestamp,
                 &point.origin.x, &point.origin.y, &point.origin.z,
                 &point.orientation.x, &point.orientation.y, &point.orientation.z, &point.orientation.w) != 8)
      {
        return false;
      }

      return true;
    };
  }
  else
  {
    _imp->trajectoryReader = new liblas::Reader(_imp->lasReaderFactory->CreateWithStream(_imp->trajectoryFile));
    _imp->readTrajectoryPoint = [this](TrajectoryPoint &point) -> bool
    {
      if (_imp->trajectoryReader->ReadNextPoint())
      {
        const liblas::Point &p = _imp->trajectoryReader->GetPoint();
        point.timestamp = p.GetTime();
        point.origin = glm::dvec3(p.GetX(), p.GetY(), p.GetZ());
        point.orientation = glm::dquat();
        return true;
      }
      return false;
    };
  }
  _imp->numberOfPoints = _imp->sampleReader->GetHeader().GetPointRecordsCount();

  // Prime the trajectory buffer.
  bool trajectoryPrimed = _imp->readTrajectoryPoint(_imp->trajectoryBuffer[0]);
  if (!trajectoryPrimed && textTrajectory)
  {
    // Try a second time to read the first trajectory point from text file. First line may be headings.
    trajectoryPrimed = _imp->readTrajectoryPoint(_imp->trajectoryBuffer[0]);
  }
  trajectoryPrimed = trajectoryPrimed && _imp->readTrajectoryPoint(_imp->trajectoryBuffer[1]);

  if (!trajectoryPrimed)
  {
    close();
    return false;
  }

  return true;
}


void SlamCloudLoader::close()
{
  delete _imp->trajectoryReader;
  delete _imp->sampleReader;
  _imp->sampleReader = _imp->trajectoryReader = nullptr;
  _imp->sampleFile.close();
  _imp->trajectoryFile.close();
  _imp->sampleFilePath.clear();
  _imp->trajectoryFilePath.clear();
  _imp->readTrajectoryPoint = [](TrajectoryPoint &) { return false; };
  _imp->numberOfPoints = 0;
}


size_t SlamCloudLoader::numberOfPoints() const
{
  return _imp->numberOfPoints;
}


bool SlamCloudLoader::realTimeMode() const
{
  return _imp->realTimeMode;
}


bool SlamCloudLoader::sampleFileIsOpen() const
{
  return _imp->sampleFile.is_open();
}


bool SlamCloudLoader::trajectoryFileIsOpen() const
{
  return _imp->trajectoryFile.is_open();
}


void SlamCloudLoader::preload(size_t pointCount)
{
  if (pointCount)
  {
    _imp->samplesBuffer.reserve(pointCount);
    while (pointCount && loadPoint())
    {
      --pointCount;
    }
  }
  else
  {
    while (loadPoint());
  }
}


bool SlamCloudLoader::nextPoint(glm::dvec3 &sample, glm::dvec3 *origin, double *timestampOut, glm::dquat *orientation)
{
  loadPoint();
  if (_imp->nextSampleReadIndex < _imp->samplesBuffer.size())
  {
    // Read next sample.
    SamplePoint samplePoint = _imp->samplesBuffer[_imp->nextSampleReadIndex++];
    //if (_imp->nextSampleReadIndex == _imp->samplesBuffer.size() || _imp->nextSampleReadIndex >= 1024)
    //{
    //  // Shrink the remaining points.
    //  const size_t newSize = (_imp->nextSampleReadIndex < _imp->samplesBuffer.size()) ? _imp->samplesBuffer.size() - _imp->nextSampleReadIndex : 0;
    //  if (_imp->nextSampleReadIndex < _imp->samplesBuffer.size())
    //  {
    //    memmove(_imp->samplesBuffer.data(),
    //            _imp->samplesBuffer.data() + _imp->nextSampleReadIndex,
    //            sizeof(*_imp->samplesBuffer.data()) * newSize
    //            );
    //  }
    //  _imp->samplesBuffer.resize(newSize);
    //  _imp->nextSampleReadIndex = 0;
    //}
    sample = samplePoint.sample;
    if (timestampOut)
    {
      *timestampOut = samplePoint.timestamp;
    }
    if (origin)
    {
      *origin = samplePoint.origin;
    }
    if (orientation)
    {
      *orientation = samplePoint.orientation;
    }

    // If in real time mode, sleep until we should deliver this sample.
    if (_imp->realTimeMode && _imp->firstSampleTimestamp >= 0)
    {
      double sampleRelativeTime = samplePoint.timestamp - _imp->firstSampleTimestamp;
      if (sampleRelativeTime > 0)
      {
        auto uptime = clock::now() - _imp->firstSampleReadTime;
        double sleepTime = sampleRelativeTime - std::chrono::duration_cast<std::chrono::duration<double>>(uptime).count();
        if (sleepTime > 0)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }
      }
    }
    return true;
  }
  return false;
}


bool SlamCloudLoader::loadPoint()
{
  if (_imp->sampleReader && _imp->sampleReader->ReadNextPoint())
  {
    const liblas::Point &p = _imp->sampleReader->GetPoint();
    SamplePoint sample;
    sample.timestamp = p.GetTime();
    sample.sample = glm::dvec3(p.GetX(), p.GetY(), p.GetZ());
    sample.origin = glm::dvec3(0);
    sample.orientation = glm::dquat();
    sampleTrajectory(sample.origin, sample.orientation, sample.timestamp);
    _imp->samplesBuffer.push_back(sample);

    if (_imp->firstSampleTimestamp < 0)
    {
      _imp->firstSampleTimestamp = sample.timestamp;
      _imp->firstSampleReadTime = clock::now();
    }
    return true;
  }
  return false;
}


bool SlamCloudLoader::sampleTrajectory(glm::dvec3 &position, glm::dquat &orientation, double timestamp)
{
  if (_imp->readTrajectoryPoint)
  {
    TrajectoryPoint trajPoint;

    while (timestamp > _imp->trajectoryBuffer[1].timestamp && _imp->readTrajectoryPoint(trajPoint))
    {
      _imp->trajectoryBuffer[0] = _imp->trajectoryBuffer[1];
      _imp->trajectoryBuffer[1] = trajPoint;
    }

    if (_imp->trajectoryBuffer[0].timestamp <= timestamp &&
        timestamp <= _imp->trajectoryBuffer[1].timestamp &&
        _imp->trajectoryBuffer[0].timestamp != _imp->trajectoryBuffer[1].timestamp)
    {
      const double lerp = (timestamp - _imp->trajectoryBuffer[0].timestamp) / (_imp->trajectoryBuffer[1].timestamp - _imp->trajectoryBuffer[0].timestamp);
      position = _imp->trajectoryBuffer[0].origin + lerp * (_imp->trajectoryBuffer[1].origin - _imp->trajectoryBuffer[0].origin);
      orientation = glm::slerp(_imp->trajectoryBuffer[0].orientation, _imp->trajectoryBuffer[1].orientation, lerp);
      return true;
    }
  }
  return false;
}
