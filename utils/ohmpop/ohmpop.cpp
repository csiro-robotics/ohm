//
// author Kazys Stepanas
//
#include "ohmpopconfig.h"

#include <glm/glm.hpp>

#include <slamio/slamcloudloader.h>

#include <ohm/ohmclearanceprocess.h>
#include <ohm/ohmmapper.h>
#include <ohm/ohmmappingprocess.h>
#include <ohm/occupancygpu.h>
#include <ohm/occupancygpumap.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancymapserialise.h>
#include <ohm/occupancynode.h>
#include <ohm/occupancytype.h>
#include <ohm/occupancyutil.h>
#include <ohmutil/ohmutil.h>
#include <ohmutil/plymesh.h>
#include <ohmutil/progressmonitor.h>
#include <ohmutil/safeio.h>
#include <ohmutil/scopedtimedisplay.h>

#include <ohmutil/options.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <thread>

#define COLLECT_STATS 0
#define COLLECT_STATS_IGNORE_FIRST 1

namespace
{
  int quit = 0;

  void onSignal(int arg)
  {
    if (arg == SIGINT || arg == SIGTERM)
    {
      ++quit;
    }
  }


  // Min/max P: 0.1192 0.971
  // Min/max V: -2.00003 3.51103
  struct Options
  {
    std::string cloudFile;
    std::string trajectoryFile;
    std::string outputBaseName;
    glm::u8vec3 regionVoxelDim;
    uint64_t pointLimit = 0;
    int64_t preloadCount = 0;
    double startTime = 0;
    double timeLimit = 0;
    double resolution = 0.25;
    double progressiveMappingSlice = 0.001;
    double mappingInterval = 0.2;
    float probHit = 0.9f;
    float probMiss = 0.49f;
    float probThresh = 0.5f;
    float clearance = 0.0f;
    glm::vec2 probRange = glm::vec2(0, 0);
    unsigned batchSize = 2048;
    bool postPopulationMapping = true;
    bool serialise = true;
    bool saveInfo = false;
    bool quiet = false;

    void print(std::ostream **out, const ohm::OccupancyMap &map) const;
  };


  void Options::print(std::ostream **out, const ohm::OccupancyMap &map) const
  {
    while (*out)
    {
      **out << "Cloud: " << cloudFile;
      if (!trajectoryFile.empty())
      {
        **out << " + " << trajectoryFile << '\n';
      }
      else
      {
        **out << " (no trajectory)\n";
      }
      if (preloadCount)
      {
        **out << "Preload: ";
        if (preloadCount < 0)
        {
          **out << "all";
        }
        else
        {
          **out << preloadCount;
        }
        **out << '\n';
      }

      if (pointLimit)
      {
        **out << "Maximum point: " << pointLimit << '\n';
      }

      if (startTime)
      {
        **out << "Process from timestamp: " << startTime << '\n';
      }

      if (timeLimit)
      {
        **out << "Process to timestamp: " << timeLimit << '\n';
      }

      std::string memSizeString;
      util::makeMemoryDisplayString(memSizeString, ohm::OccupancyMap::nodeMemoryPerRegion(regionVoxelDim));
      **out << "Map resolution: " << resolution << '\n';
      glm::i16vec3 regionDim = regionVoxelDim;
      regionDim.x = (regionDim.x) ? regionDim.x : OHM_DEFAULT_CHUNK_DIM_X;
      regionDim.y = (regionDim.y) ? regionDim.y : OHM_DEFAULT_CHUNK_DIM_Y;
      regionDim.z = (regionDim.z) ? regionDim.z : OHM_DEFAULT_CHUNK_DIM_Z;
      **out << "Map region dimensions: " << regionDim << '\n';
      **out << "Map region memory: " << memSizeString << '\n';
      **out << "Hit probability: " << probHit << '\n';
      **out << "Miss probability: " << probMiss << '\n';
      **out << "Probability range: [" << map.minNodeProbability() << ' ' << map.maxNodeProbability() << "]\n";
      **out << "Ray batch size: " << batchSize << '\n';
      **out << "Clearance mapping: ";
      if (clearance > 0)
      {
        **out << clearance << "m range\n";
      }
      else
      {
        **out << "disabled\n";
      }

      **out << "Mapping mode: ";
      if (progressiveMappingSlice)
      {
        **out << "progressive time slice " << progressiveMappingSlice << "s\n";
        **out << "Mapping interval: " << mappingInterval << "s\n";
        **out << "Post population mapping: " << (postPopulationMapping ? "on" : "off") << '\n';
      }
      else
      {
        **out << "post" << '\n';
      }

      **out << std::flush;

      ++out;
    }
  }

  class SaveMapProgress : public ohm::SerialiseProgress
  {
  public:
    SaveMapProgress(ProgressMonitor &monitor)
      : _monitor(monitor)
    {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { _monitor.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc = 1) override { _monitor.incrementProgressBy(inc); }

  private:
    ProgressMonitor &_monitor;
  };


  class MapperThread
  {
  public:
    MapperThread(ohm::OccupancyMap &map, const Options &opt);
    ~MapperThread();

    void start();
    void join(bool waitForCompletion = true);

  private:
    void run();

    ohm::OccupancyMap &_map;
    ohm::Mapper _mapper;
    std::thread *_thread = nullptr;
    double _timeSliceSec = 0.001;
    double _intervalSec = 0.0;
    std::atomic_bool _allowCompletion;
    std::atomic_bool _quitRequest;
  };

  MapperThread::MapperThread(ohm::OccupancyMap &map, const Options &opt)
    : _map(map)
    , _timeSliceSec(opt.progressiveMappingSlice)
    , _intervalSec(opt.mappingInterval)
    , _allowCompletion(true)
    , _quitRequest(false)
  {
    _mapper.setMap(&map);
    if (opt.clearance > 0)
    {
      _mapper.addProcess(new ohm::ClearanceProcess(opt.clearance, ohm::QF_GpuEvaluate));
    }
  }


  MapperThread::~MapperThread()
  {
    join(false);
    delete _thread;
  }


  void MapperThread::start()
  {
    if (!_thread)
    {
      _thread = new std::thread(std::bind(&MapperThread::run, this));
    }
  }


  void MapperThread::join(bool waitForCompletion)
  {
    if (_thread)
    {
      _allowCompletion = waitForCompletion;
      _quitRequest = true;
      _thread->join();
      delete _thread;
      _thread = nullptr;
    }
  }


  void MapperThread::run()
  {
    using Clock = std::chrono::high_resolution_clock;
    while (!_quitRequest)
    {
      const auto loopStart = Clock::now();
      if (_timeSliceSec > 0)
      {
        _mapper.update(_timeSliceSec);
      }
      if (_intervalSec > 0)
      {
        std::this_thread::sleep_until(loopStart + std::chrono::duration<double>(_intervalSec));
      }
    }

    if (_allowCompletion)
    {
      _mapper.update(0);
    }
  }
}


int populateMap(const Options &opt)
{
  ohmutil::ScopedTimeDisplay timeDisplay("Execution time");
  if (opt.quiet)
  {
    timeDisplay.disable();
  }

  std::cout << "Loading points from " << opt.cloudFile << " with trajectory " << opt.trajectoryFile << std::endl;

  SlamCloudLoader loader;
  if (!loader.open(opt.cloudFile.c_str(), opt.trajectoryFile.c_str()))
  {
    fprintf(stderr, "Error loading cloud %s with trajectory %s \n", opt.cloudFile.c_str(), opt.trajectoryFile.c_str());
    return -2;
  }

  using Clock = std::chrono::high_resolution_clock;
  ohm::OccupancyMap map(opt.resolution, opt.regionVoxelDim);
  ohm::GpuMap gpuMap(&map, true, opt.batchSize);
  //MapperThread mapper(map, opt);
  ohm::Mapper mapper(&map);
  std::vector<double> sampleTimestamps;
  std::vector<glm::dvec3> originSamplePairs;
  glm::dvec3 origin, sample;
  // glm::vec3 voxel, ext(opt.resolution);
  double timestamp;
  double mappingInterval = opt.mappingInterval;
  uint64_t pointCount = 0;
  // Update map visualisation every N samples.
  const size_t rayBatchSize = opt.batchSize;
  double timebase = -1;
  double firstTimestamp = -1;
  double lastTimestamp = -1;
  double firstBatchTimestamp = -1;
  double nextMapperUpdate = opt.mappingInterval;
  std::atomic<uint64_t> elapsedMs(0);
  Clock::time_point startTime, endTime;
  ProgressMonitor prog(10);

  if (!gpuMap.gpuOk())
  {
    std::cerr << "Failed to initialise GpuMap programs." << std::endl;
    return -3;
  }

  map.setHitProbability(opt.probHit);
  map.setOccupancyThresholdProbability(opt.probThresh);
  map.setMissProbability(opt.probMiss);
  if (opt.probRange[0])
  {
    map.setMinNodeProbability(opt.probRange[0]);
  }
  if (opt.probRange[1])
  {
    map.setMaxNodeProbability(opt.probRange[1]);
  }
  // map.setSaturateAtMinValue(opt.saturateMin);
  // map.setSaturateAtMaxValue(opt.saturateMax);

  // Prevent ready saturation to free.
  // map.setClampingThresMin(0.01);
  // printf("min: %g\n", map.getClampingThresMinLog());

  if (opt.clearance > 0)
  {
    mapper.addProcess(new ohm::ClearanceProcess(opt.clearance, ohm::QF_GpuEvaluate));
  }

  std::ostream *streams[] = { &std::cout, nullptr, nullptr };
  std::ofstream infoStream;
  if (opt.saveInfo)
  {
    streams[1] = &infoStream;
    std::string outputFile = opt.outputBaseName + ".txt";
    std::ofstream out(outputFile.c_str());
    infoStream.open(outputFile.c_str());
  }

  opt.print(streams, map);

  if (opt.preloadCount)
  {
    int64_t preloadCount = opt.preloadCount;
    if (preloadCount < 0 && opt.pointLimit)
    {
      preloadCount = opt.pointLimit;
    }

    std::cout << "Preloading points";

    startTime = Clock::now();
    if (preloadCount < 0)
    {
      std::cout << std::endl;
      loader.preload();
    }
    else
    {
      std::cout << " " << preloadCount << std::endl;
      loader.preload(preloadCount);
    }
    endTime = Clock::now();
    const double preloadTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() * 1e-3;
    std::cout << "Preload completed over " << preloadTime << " seconds." << std::endl;
  }

  startTime = Clock::now();
  std::cout << "Populating map" << std::endl;

  prog.setDisplayFunction([&elapsedMs, &opt](const ProgressMonitor::Progress &prog) {
    if (!opt.quiet)
    {
      const uint64_t elapsedMsLocal = elapsedMs;
      const uint64_t sec = elapsedMsLocal / 1000u;
      const unsigned ms = unsigned(elapsedMsLocal - sec * 1000);

      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (prog.info.info && prog.info.info[0])
      {
        out << prog.info.info << " : ";
      }

      out << sec << '.' << std::setfill('0') << std::setw(3) << ms << "s : ";

      out << std::setfill(' ') << std::setw(12) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(12) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });
  prog.beginProgress(ProgressMonitor::Info(
    (pointCount && timebase == 0) ? std::min<uint64_t>(pointCount, loader.numberOfPoints()) : loader.numberOfPoints()));
  prog.startThread();

#if COLLECT_STATS
  struct TimeStats
  {
    uint64_t totalNs = 0;
    uint64_t maxNs = 0;
    uint64_t updateCount = 0;

    void add(const Clock::duration &elapsed)
    {
      const uint64_t timeNs = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
      maxNs = std::max(timeNs, maxNs);
      totalNs += timeNs;
      ++updateCount;
    }

    void print() const
    {
      std::ostringstream str;
      str << "\n*************************************" << std::endl;
      const std::chrono::nanoseconds avgTNs(totalNs / updateCount);
      str << "Average integration time: " << avgTNs << std::endl;
      const std::chrono::nanoseconds maxTNs(maxNs);
      str << "Max integration time: " << maxTNs << std::endl;
      str << "*************************************" << std::endl;
      std::cout << str.str() << std::flush;
    }
  };

  TimeStats stats;
#endif // COLLECT_STATS

  //------------------------------------
  // Population loop.
  //------------------------------------
  //mapper.start();
  origin = glm::vec3(0, 0, 0);
  while ((pointCount < opt.pointLimit || opt.pointLimit == 0) &&
    (lastTimestamp - timebase < opt.timeLimit || opt.timeLimit == 0) &&
    loader.nextPoint(sample, &origin, &timestamp))
  {
    if (timebase < 0)
    {
      timebase = timestamp;
    }

    if (timestamp - timebase < opt.startTime)
    {
      continue;
    }

    if (lastTimestamp < 0)
    {
      lastTimestamp = timestamp;
    }

    if (firstTimestamp < 0)
    {
      firstTimestamp = timestamp;
    }

    ++pointCount;
    sampleTimestamps.push_back(timestamp);
    originSamplePairs.push_back(origin);
    originSamplePairs.push_back(sample);

    if (firstBatchTimestamp < 0)
    {
      firstBatchTimestamp = timestamp;
    }

    if (pointCount % rayBatchSize == 0 || quit)
    {
#if COLLECT_STATS
      const auto then = Clock::now();
#endif // COLLECT_STATS
      gpuMap.integrateRays(originSamplePairs.data(), unsigned(originSamplePairs.size()));
#if COLLECT_STATS
      const auto integrateTime = Clock::now() - then;
#if COLLECT_STATS_IGNORE_FIRST
      if (firstBatchTimestamp != timestamp)
#endif // COLLECT_STATS_IGNORE_FIRST
      {
        stats.add(integrateTime);
      }
      const unsigned kLongUpdateThresholdMs = 100u;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(integrateTime).count() > kLongUpdateThresholdMs)
      {
        std::ostringstream str;
        str << '\n' << sampleTimestamps.front() - firstTimestamp << " (" << sampleTimestamps.front() << "): long upate " << integrateTime << std::endl;
        std::cout << str.str() << std::flush;
  }
#endif // COLLECT_STATS
      sampleTimestamps.clear();
      originSamplePairs.clear();

      const double elapsedTime = timestamp - lastTimestamp;
      firstBatchTimestamp = -1;

      prog.incrementProgressBy(rayBatchSize);
      lastTimestamp = timestamp;
      // Store into elapsedMs atomic.
      elapsedMs = uint64_t((lastTimestamp - timebase) * 1e3);

      if (opt.progressiveMappingSlice > 0)
      {
        if (opt.mappingInterval >= 0)
        {
          nextMapperUpdate -= elapsedTime;
        }
        if (nextMapperUpdate <= 0)
        {
          nextMapperUpdate += opt.mappingInterval;
          const auto mapperStart = Clock::now();
          mapper.update(opt.progressiveMappingSlice);
          const auto mapperEnd = Clock::now();
          std::ostringstream msg;
          msg << "\nMapper: " << (mapperEnd - mapperStart) << '\n';
          std::cout << msg.str();
        }
      }

      if (opt.pointLimit && pointCount >= opt.pointLimit ||
          opt.timeLimit && lastTimestamp - timebase >= opt.timeLimit || quit)
      {
        break;
      }
    }
  }

  // Make sure we have no more rays.
  if (!originSamplePairs.empty())
  {
#if COLLECT_STATS
    const auto then = Clock::now();
#endif // COLLECT_STATS
    gpuMap.integrateRays(originSamplePairs.data(), unsigned(originSamplePairs.size()));
#if COLLECT_STATS
    const auto integrateTime = Clock::now() - then;
    stats.add(integrateTime);
#endif // COLLECT_STATS
    sampleTimestamps.clear();
    originSamplePairs.clear();
  }

  prog.endProgress();
  prog.pause();

  const auto mapperStart = Clock::now();
  if (opt.postPopulationMapping && !quit)
  {
    mapper.update(0.0);
  }
  //mapper.join(!quit && opt.postPopulationMapping);
  endTime = Clock::now();

#if COLLECT_STATS
  stats.print();
#endif // COLLECT_STATS

  if (!opt.quiet)
  {
    std::cout << std::endl;
  }

  // Sync the map.
  if (!opt.quiet)
  {
    std::cout << "syncing map" << std::endl;
  }
  gpuMap.syncOccupancy();

  const double processingTimeSec =
    std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() * 1e-3;

  std::ostream **out = streams;
  while (*out)
  {
    const double timeRange = lastTimestamp - firstTimestamp;
    **out << "Point count: " << pointCount << '\n';
    **out << "Data time: " << timeRange << '\n';
    **out << "Population completed in " << mapperStart - startTime << std::endl;
    **out << "Post mapper completed in " << endTime - mapperStart << std::endl;
    **out << "Total processing time: " << endTime - startTime << '\n';
    **out << "Efficiency: " << ((timeRange) ? processingTimeSec / timeRange : 0.0) << '\n';
    **out << "Points/sec: " << ((processingTimeSec > 0) ? pointCount / processingTimeSec : 0.0) << '\n';
    **out << "Memory (approx): " << map.calculateApproximateMemory() / (1024.0 * 1024.0) << " MiB\n";
    **out << std::flush;
    ++out;
  }

  if (opt.serialise)
  {
    if (quit < 2)
    {
      std::string outputFile = opt.outputBaseName + ".ohm";
      std::cout << "Saving map to " << outputFile.c_str() << std::endl;
      SaveMapProgress saveProgress(prog);
      prog.unpause();
      int err = ohm::save(outputFile.c_str(), map, &saveProgress);
      prog.endProgress();
      if (!opt.quiet)
      {
        std::cout << std::endl;
      }

      if (err)
      {
        fprintf(stderr, "Failed to save map: %d\n", err);
      }
    }

    // Save a cloud representation.
    std::cout << "Converting to point cloud." << std::endl;
    PlyMesh ply;
    glm::vec3 v;
    const auto mapEndIter = map.end();
    const size_t regionCount = map.regionCount();
    glm::i16vec3 lastRegion = map.begin().key().regionKey();
    pointCount = 0;

    prog.beginProgress(ProgressMonitor::Info(regionCount));

    if (opt.serialise)
    {
      for (auto iter = map.begin(); iter != mapEndIter && quit < 2; ++iter)
      {
        const ohm::OccupancyNodeConst node = *iter;
        if (lastRegion != iter.key().regionKey())
        {
          prog.incrementProgress();
          lastRegion = iter.key().regionKey();
        }
        if (node.isOccupied())
        {
          v = map.voxelCentreLocal(node.key());
          ply.addVertex(v);
          ++pointCount;
        }
      }

      prog.endProgress();
      prog.pause();
      if (!opt.quiet)
      {
        std::cout << "\nExported " << pointCount << " point(s)" << std::endl;
      }

      if (quit < 2)
      {
        std::string outputFile = opt.outputBaseName + ".ply";
        std::cout << "Saving point cloud to " << outputFile.c_str() << std::endl;
        ply.save(outputFile.c_str(), true);
      }
    }
  }

  prog.joinThread();

  return 0;
}


namespace
{
  void printOptions(std::ostream **out, const Options &opt, const ohm::OccupancyMap &map)
  {
    while (*out)
    {
      **out << "Cloud: " << opt.cloudFile;
      if (!opt.trajectoryFile.empty())
      {
        **out << " + " << opt.trajectoryFile << '\n';
      }
      else
      {
        **out << " (no trajectory)\n";
      }
      if (opt.preloadCount)
      {
        **out << "Preload: ";
        if (opt.preloadCount < 0)
        {
          **out << "all";
        }
        else
        {
          **out << opt.preloadCount;
        }
        **out << '\n';
      }

      if (opt.pointLimit)
      {
        **out << "Maximum point: " << opt.pointLimit << '\n';
      }

      if (opt.startTime)
      {
        **out << "Process from timestamp: " << opt.startTime << '\n';
      }

      if (opt.timeLimit)
      {
        **out << "Process to timestamp: " << opt.timeLimit << '\n';
      }

      std::string memSizeString;
      util::makeMemoryDisplayString(memSizeString, ohm::OccupancyMap::nodeMemoryPerRegion(opt.regionVoxelDim));
      **out << "Map resolution: " << opt.resolution << '\n';
      glm::i16vec3 regionDim = opt.regionVoxelDim;
      regionDim.x = (regionDim.x) ? regionDim.x : OHM_DEFAULT_CHUNK_DIM_X;
      regionDim.y = (regionDim.y) ? regionDim.y : OHM_DEFAULT_CHUNK_DIM_Y;
      regionDim.z = (regionDim.z) ? regionDim.z : OHM_DEFAULT_CHUNK_DIM_Z;
      **out << "Map region dimensions: " << regionDim << '\n';
      **out << "Map region memory: " << memSizeString << '\n';
      **out << "Hit probability: " << opt.probHit << " (" << ohm::probabilityToValue(opt.probHit) << ")\n";
      **out << "Miss probability: " << opt.probMiss << " (" << ohm::probabilityToValue(opt.probMiss) << ")\n";
      **out << "Occupancy threshold: " << opt.probThresh << " (" << ohm::probabilityToValue(opt.probThresh) << ")\n";
      **out << "Probability range: [" << map.minNodeProbability() << ' ' << map.maxNodeProbability() << "]\n";
      **out << "Ray batch size: " << opt.batchSize << '\n';

      ++out;
    }
  }
}

int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options optParse(argv[0],
                            "Generate an occupancy map from a LAS/LAZ based point cloud and accompanying "
                            "trajectory file using GPU. The trajectory marks the scanner trajectory with timestamps "
                            "loosely corresponding to cloud point timestamps. Trajectory points are "
                            "interpolated for each cloud point based on corresponding times in the "
                            "trajectory.");
  optParse.positional_help("<cloud.laz> <_traj.txt> [output-base]");

  try
  {
    // Build GPU options set.
    std::vector<int> gpuOptionsTypes(ohm::gpuArgsInfo(nullptr, nullptr, 0));
    std::vector<const char *> gpuOptions(gpuOptionsTypes.size() * 2);
    ohm::gpuArgsInfo(gpuOptions.data(), gpuOptionsTypes.data(), unsigned(gpuOptionsTypes.size()));

    // clang-format off
    optParse.add_options()
      ("b,batch-size", "The number of points to process in each batch. Controls debug display.", optVal(opt.batchSize))
      ("help", "Show help.")
      ("i,cloud", "The input cloud (las/laz) to load.", cxxopts::value(opt.cloudFile))
      ("o,output","Output base name", optVal(opt.outputBaseName))
      ("p,point-limit", "Limit the number of points loaded.", optVal(opt.pointLimit))
      ("preload", "Preload this number of points before starting processing. Zero for all. May be used for separating processing and loading time.", optVal(opt.preloadCount)->default_value("0"))
      ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(opt.quiet))
      ("s,start-time", "Only process points time stamped later than the specified time.", optVal(opt.startTime))
      ("save-info", "Save timing information to text based on the output file name.", optVal(opt.saveInfo))
      ("serialise", "Serialise the results? This option is intended for skipping saving during performance analysis.", optVal(opt.serialise))
      ("t,time-limit", "Limit the elapsed time in the LIDAR data to process (seconds). Measured relative to the first data sample.", optVal(opt.timeLimit))
      ("trajectory", "The trajectory (text) file to load.", cxxopts::value(opt.trajectoryFile))
      ;

    optParse.add_options("Map")
      ("clamp", "Set probability clamping to the given min/max.", optVal(opt.probRange))
      ("d,dim", "Set the voxel dimensions of each region in the map. Range for each is [0, 255).", optVal(opt.regionVoxelDim))
      ("h,hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(opt.probHit))
      ("m,miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(opt.probMiss))
      ("r,resolution", "The voxel resolution of the generated map.", optVal(opt.resolution))
      ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(opt.probThresh)->implicit_value(optStr(opt.probThresh)))
      ;

    optParse.add_options("Mapping")
      ("clearance", "Calculate clearance values for the map using this as the maximum search range. Zero to disable.", optVal(opt.clearance))
      ("progressive", "Time slice allowed for progressive mapping processes. Zero to disable and update after population.", optVal(opt.progressiveMappingSlice))
      ("progressive-interval", "Interval for progressive mapping. Time is based on input data time.", cxxopts::value(opt.mappingInterval)->default_value(optStr(opt.mappingInterval)))
      ("post-mapping", "Allow mapping thread to complete after population?", optVal(opt.postPopulationMapping))
      ;

    // clang-format on

    if (!gpuOptions.empty())
    {
      auto adder = optParse.add_options("GPU");
      for (size_t i = 0; i < gpuOptionsTypes.size(); ++i)
      {
        adder(gpuOptions[(i << 1) + 0], gpuOptions[(i << 1) + 1],
              gpuOptionsTypes[i] == 0 ? ::cxxopts::value<bool>() : ::cxxopts::value<std::string>());
      }
    }


    optParse.parse_positional({ "cloud", "trajectory", "output" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Map", "Mapping", "GPU" }) << std::endl;
      return 1;
    }

    if (opt.cloudFile.empty())
    {
      std::cerr << "Missing input cloud" << std::endl;
      return -1;
    }
    if (opt.trajectoryFile.empty())
    {
      std::cerr << "Missing trajectory file" << std::endl;
      return -1;
    }
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    return -1;
  }

  return 0;
}

int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = 0;
  res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  // Generate output name based on input if not specified.
  if (opt.outputBaseName.empty())
  {
    auto extensionStart = opt.cloudFile.find_last_of(".");
    if (extensionStart != std::string::npos)
    {
      opt.outputBaseName = opt.cloudFile.substr(0, extensionStart);
    }
    else
    {
      opt.outputBaseName = opt.cloudFile;
    }
  }

  res = ohm::configureGpuFromArgs(argc, argv);
  if (res)
  {
    return res;
  }

  res = populateMap(opt);
  return res;
}
