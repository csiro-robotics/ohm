//
// author Kazys Stepanas
//
#include "OhmQueryConfig.h"

#include <glm/glm.hpp>
#include <3esservermacros.h>

#include <ohm/OhmGpu.h>
#include <ohm/GpuMap.h>
#include <ohm/LineQuery.h>
#include <ohm/OccupancyMap.h>
#include <ohm/MapSerialise.h>
#include <ohm/NearestNeighbours.h>
#include <ohm/Voxel.h>
#include <ohm/OccupancyType.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/QueryFlag.h>
#include <ohm/OccupancyType.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/GlmStream.h>
#include <ohmutil/SafeIO.h>

#include <ohm/DebugIDs.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <locale>
#include <iostream>
#include <sstream>

typedef std::chrono::high_resolution_clock TimingClock;

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
  struct Options
  {
    struct Neighbours
    {
      glm::dvec3 point;
      float radius = -1;
    };

    struct Line
    {
      glm::dvec3 start, end;
      float radius = -1;
    };

    struct Ranges
    {
      glm::dvec3 min, max;
      float radius = -1;
    };

    std::string map_file;
    std::string output_base;
    Neighbours neighbours;
    Ranges ranges;
    Line line;
    int repeat = 0;
    bool unknown_as_occupied = true;
    bool use_gpu = false;
    bool gpu_compare = false;
    bool hard_reset_on_repeat = false;
    bool quiet = false;

    inline bool haveQuery() const
    {
      return neighbours.radius > 0 || line.radius > 0 || ranges.radius > 0;
    }

    void print() const;
  };


  void Options::print() const
  {
    std::cout << "Map: " << map_file << std::endl;
    std::cout << "Output: " << output_base << std::endl;
    if (neighbours.radius >= 0)
    {
      std::cout << "Nearest neighbours: " << neighbours.point << " R: " << neighbours.radius << std::endl;
    }
    if (line.radius >= 0)
    {
      std::cout << "Line " << line.start << "->" << line.end << " R: " << line.radius << std::endl;
    }
    if (ranges.radius >= 0)
    {
      std::cout << "Ranges: " << ranges.min << "->" << ranges.max << " R: " << ranges.radius << std::endl;
    }
  }



  class LoadMapProgress : public ohm::SerialiseProgress
  {
  public:
    LoadMapProgress(ProgressMonitor &monitor) : monitor_(monitor) {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc = 1) override { monitor_.incrementProgressBy(inc);  }

  private:
    ProgressMonitor &monitor_;
  };
}


inline std::istream &operator>>(std::istream &in, Options::Neighbours &n)
{
  glm::dvec4 v;
  //::operator>>(in, v);
  in >> v;
  n.point[0] = v[0];
  n.point[1] = v[1];
  n.point[2] = v[2];
  n.radius = float(v[3]);
  return in;
}

inline std::ostream &operator<<(std::ostream &out, const Options::Neighbours &n)
{
  out << n.point[0] << ',' << n.point[1] << ',' << n.point[2] << ',' << n.radius;
  return out;
}

inline std::istream &operator>>(std::istream &in, Options::Line &l)
{
  double v[7];
  parseVector(in, v, 7);
  l.start[0] = v[0];
  l.start[1] = v[1];
  l.start[2] = v[2];
  l.end[0] = v[3];
  l.end[1] = v[4];
  l.end[2] = v[5];
  l.radius = float(v[6]);
  return in;
}

inline std::ostream &operator<<(std::ostream &out, const Options::Line &l)
{
  out << l.start[0] << ',' << l.start[1] << ',' << l.start[2] << ',' << l.end[0] << ',' << l.end[1] << ',' << l.end[2] << ',' << l.radius;
  return out;
}

inline std::istream &operator>>(std::istream &in, Options::Ranges &r)
{
  double v[7];
  parseVector(in, v, 7);
  r.min[0] = v[0];
  r.min[1] = v[1];
  r.min[2] = v[2];
  r.max[0] = v[3];
  r.max[1] = v[4];
  r.max[2] = v[5];
  r.radius = float(v[6]);
  return in;
}

inline std::ostream &operator<<(std::ostream &out, const Options::Ranges &r)
{
  out << r.min[0] << ',' << r.min[1] << ',' << r.min[2] << ',' << r.max[0] << ',' << r.max[1] << ',' << r.max[2] << ',' << r.radius;
  return out;
}

// This is messy :(
// Must come after streaming operators for custom command line arguments are defined.
#include <ohmutil/Options.h>

int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0],
"\nLoads an occupancy map file and runs a single query on the map, exporting the\n"
"results to a new PLY cloud. trajectory file. The trajectory marks the scanner\n"
"trajectory with timestamps loosely corresponding to cloud point timestamps.\n"
"Trajectory points are interpolated for each cloud point based on corresponding\n"
"times in the trajectory."
  );
  opt_parse.positional_help("<map.ohm> <--near=x,y,z,r | --line=x1,y1,z1,x2,y2,z2,r | --ranges=x1,y1,z1,x2,y2,z2,r>");

  try
  {
    // clang-format off
    opt_parse.add_options()
      ("gpu", "Use GPU based queries where possible", optVal(opt.use_gpu))
      ("q,quiet", "Run in quiet mode. Suppresses progress messages.", optVal(opt.use_gpu))
      ("gpu-compare", "Compare CPU and GPU results for the query. Implies '--gpu'.", optVal(opt.gpu_compare))
      ("hard-reset", "Perform a hard reset when repeatedly executing a query (--repeat option). Soft reset is the default.", optVal(opt.hard_reset_on_repeat))
      ("o,output", "Sets the base PLY file names to save results to. Defaults to <<mapfile>-near.ply> and <<mapfile>-line.ply>", optVal(opt.output_base))
      ("line", "Perform a line segment test from (x1,y1,z1) to (x2,y2,z2) considering voxels withing radius r of the line segment.", optVal(opt.line), "x1,y1,z1,x2,y2,z2,r")
      ("near", "Perform a nearest neighbours query at the point (x,y,z) with a radius of r.", optVal(opt.neighbours), "x,y,z,r")
      ("ranges", "Calculate the nearest occupied voxel for each voxel in the specified min/max extents, (x1,y1,z1) to (x2,y2,z2).", optVal(opt.ranges), "x1,y1,z1,x2,y2,z2,r")
      ("repeat", "Repeat the query N times. For timing evaluation.", optVal(opt.repeat))
      ("uao", "Treat unknown space as occupied/obstructed?", optVal(opt.unknown_as_occupied))
    ;
    // clang-format off

    opt_parse.parse_positional({ "output" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    bool ok = true;

    if (opt.map_file.empty())
    {
      ok = false;
      std::cerr << "Missing input map" << std::endl;
    }

    if (opt.ranges.min.x > opt.ranges.max.x ||
        opt.ranges.min.y > opt.ranges.max.y ||
        opt.ranges.min.z > opt.ranges.max.z)
    {
      ok = false;
      std::cerr << "Minimum range " << opt.ranges.min << " exceeds maximum " << opt.ranges.max << std::endl;
    }

    if (!ok)
    {
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

void initialiseDebugCategories(const Options &/*opt*/)
{
  // TES_CATEGORY(ohm::g_3es, "Map", CAT_Map, 0, true);
  // TES_CATEGORY(ohm::g_3es, "Populate", CAT_Populate, 0, true);
  // TES_IF(opt.rays & Rays_Lines)
  // {
  //   TES_CATEGORY(ohm::g_3es, "Rays", CAT_Rays, CAT_Populate, (opt.rays & Rays_Lines) != 0);
  // }
  // TES_IF(opt.rays & Rays_Voxels)
  // {
  //   TES_CATEGORY(ohm::g_3es, "Free", CAT_FreeCells, CAT_Populate, (opt.rays & Rays_Lines) == 0);
  // }
  // TES_IF(opt.samples)
  // {
  //   TES_CATEGORY(ohm::g_3es, "Occupied", CAT_OccupiedCells, CAT_Populate, true);
  // }
  // TES_CATEGORY(ohm::g_3es, "Info", CAT_Info, 0, true);
}

void saveQueryCloud(const ohm::OccupancyMap &map, const ohm::Query &query, const Options &opt,
                    const std::string &suffix, float colour_range = 0.0f)
{
  const size_t result_count = query.numberOfResults();
  const ohm::Key *keys = query.intersectedVoxels();
  const float *ranges = query.ranges();
  glm::dvec3 voxel_pos;

  PlyMesh ply;
  for (size_t i = 0; i < result_count; ++i)
  {
    const ohm::Key &key = keys[i];
    uint8_t c = 255;
    if (colour_range > 0 && ranges)
    {
      const float range_value = ranges[i];
      c = uint8_t(255 * std::max(0.0f, (colour_range - range_value) / colour_range));
    }
    voxel_pos = map.voxelCentreGlobal(key);
    ply.addVertex(voxel_pos, Colour(c, 128, 0));
  }

  std::string str = opt.output_base;
  str += suffix + ".ply";
  printf("Saving results to %s\n", str.c_str());
  ply.save(str.c_str(), true);
}


#ifdef FIXME
void saveRangesCloud(const ohm::OccupancyMap &map, const ohm::VoxelRanges &query, const std::string &suffix,
                     ProgressMonitor &prog, const Options &opt,
                     std::vector<glm::vec4> *pointsOut = nullptr,
                     bool clearValues = false)
{
  std::string str = opt.outputBase;
  str += suffix + ".ply";
  printf("Exporting ranges cloud to %s\n", str.c_str());
  PlyMesh ply;
  glm::vec3 v;
  auto mapEndIter = map.end();
  size_t regionCount = map.regionCount();
  glm::i16vec3 lastRegion = map.begin().key().regionKey();
  glm::i16vec3 minRegion, maxRegion;
  uint64_t pointCount = 0;

  prog.beginProgress(ProgressMonitor::Info(regionCount));

  minRegion = map.regionKey(query.minExtents());
  maxRegion = map.regionKey(query.maxExtents());

  const float colourScale = query.searchRadius();
  for (auto iter = map.begin(); iter != mapEndIter && quit < 2; ++iter)
  {
    const ohm::VoxelConst voxel = *iter;
    if (lastRegion != iter.key().regionKey())
    {
      prog.incrementProgress();
      lastRegion = iter.key().regionKey();
    }

    // Ensure the voxel is in a region we have calculated data for.
    if (minRegion.x <= lastRegion.x && lastRegion.x <= maxRegion.x &&
        minRegion.y <= lastRegion.y && lastRegion.y <= maxRegion.y &&
        minRegion.z <= lastRegion.z && lastRegion.z <= maxRegion.z)
    {
      if (voxel.isOccupied() || voxel.isFree())
      {
        const float rangeValue = voxel.clearance();
        if (rangeValue >= 0)
        {
          uint8_t c = (uint8_t)(255 * std::max(0.0f, (colourScale - rangeValue) / colourScale));
          v = map.voxelCentreLocal(voxel.key());
          ply.addVertex(v, Colour(c, 128, 0));
          if (pointsOut)
          {
            pointsOut->push_back(glm::vec4(v, rangeValue));
          }
          ++pointCount;
        }
      }

      if (clearValues && !voxel.isNull())
      {
        voxel.makeMutable().setClearance(-1.0f);
      }
    }
  }

  ply.save(str.c_str(), true);

  prog.endProgress();
  prog.pause();

  if (!opt.quiet)
  {
    printf("\nExported %" PRIu64 " points\n", pointCount);
  }
}
#endif // FIXME


void showTiming(const char *info, const TimingClock::time_point &start_time, const TimingClock::time_point &end_time,
                int cycles)
{
  std::string timing_str;
  TimingClock::duration exec_time = end_time - start_time;
  util::timeString(timing_str, exec_time);
  if (cycles <= 1)
  {
    printf("%s query completed in %s\n", info, timing_str.c_str());
  }
  else
  {
    printf("%s query completed %d queries in %s\n", info, cycles, timing_str.c_str());
    exec_time /= cycles;
    util::timeString(timing_str, exec_time);
    printf("average time per query: %s\n", timing_str.c_str());
  }
}


bool compareCpuGpuQuery(const char *query_name, ohm::Query &query, const float epsilon = 1e-5f)
{
  std::string timing_info_str;
  TimingClock::time_point query_start, query_end;

  // CPU execution.
  query.setQueryFlags(query.queryFlags() & ~ohm::kQfGpuEvaluate);
  query_start = TimingClock::now();
  query.reset();
  query.execute();
  query_end = TimingClock::now();

  std::vector<ohm::Key> keys;
  std::vector<float> ranges;

  keys.resize(query.numberOfResults());
  ranges.resize(query.numberOfResults());

  if (query.numberOfResults())
  {
    memcpy(keys.data(), query.intersectedVoxels(), sizeof(*keys.data()) * query.numberOfResults());
    memcpy(ranges.data(), query.ranges(), sizeof(*ranges.data()) * query.numberOfResults());
  }

  std::cout <<
            "Comparing CPU/GPU execution. Note GPU execution time may be better on repeated queries due to setup overhead and caching gains."
            << std::endl;
  timing_info_str = query_name;
  timing_info_str += " CPU";
  showTiming(timing_info_str.c_str(), query_start, query_end, 1);

  // GPU execution
  query.setQueryFlags(query.queryFlags() | ohm::kQfGpuEvaluate);
  query_start = TimingClock::now();
  query.reset();
  query.execute();
  query_end = TimingClock::now();

  timing_info_str = query_name;
  timing_info_str += " GPU";
  showTiming(timing_info_str.c_str(), query_start, query_end, 1);

  std::cout << "Comparing " << keys.size() << " results" << std::endl;
  if (keys.size() != query.numberOfResults())
  {
    std::cerr << "Result count mismatch (CPU/GPU): " << keys.size() << '/' << query.numberOfResults() << std::endl;
  }

  // Compare results.
  bool results_match = true;
  float range_diff;
  bool key_found;

  // Look for duplicate GPU results.
  for (size_t i = 0; i < query.numberOfResults(); ++i)
  {
    const ohm::Key key = query.intersectedVoxels()[i];
    for (size_t j = i + 1; j < query.numberOfResults(); ++j)
    {
      if (key == query.intersectedVoxels()[j])
      {
        std::cerr << "Duplicate GPU key result: ("
                  << keys[i].regionKey().x << ',' << keys[i].regionKey().y << ',' << keys[i].regionKey().z
                  << "):[" << int(keys[i].localKey().x) << ',' << int(keys[i].localKey().y) << ',' << int(keys[i].localKey().z)
                  << ']' << std::endl;
      }
    }
  }

  // Compare CPU/GPU results.
  for (size_t i = 0; i < keys.size(); ++i)
  {
    key_found = false;
    // Find the key in the GPU result. Order from GPU is non-deterministic.
    for (size_t j = 0; j < query.numberOfResults(); ++j)
    {
      if (keys[i] == query.intersectedVoxels()[j])
      {
        key_found = true;
        range_diff = ranges[i] - query.ranges()[j];
        if (std::abs(range_diff) > epsilon)
        {
          std::cerr << "Range diff for voxel ("
                    << keys[i].regionKey().x << ',' << keys[i].regionKey().y << ',' << keys[i].regionKey().z
                    << "):[" << int(keys[i].localKey().x) << ',' << int(keys[i].localKey().y) << ',' << int(keys[i].localKey().z)
                    << "]: " << ranges[i] << "/" << query.ranges()[j] << ':' << range_diff << std::endl;
          results_match = false;
        }
        break;
      }
    }

    if (!key_found)
    {
      std::cerr << "No matching GPU key for voxel ("
                << keys[i].regionKey().x << ',' << keys[i].regionKey().y << ',' << keys[i].regionKey().z
                << "):[" << int(keys[i].localKey().x) << ',' << int(keys[i].localKey().y) << ',' << int(keys[i].localKey().z)
                << ']' << std::endl;
      results_match = false;
    }
  }

  return results_match;
}


void executeQuery(const char *query_name, const Options &opt, ohm::Query &query, const float range_epsilon = 1e-5f)
{
  if (!opt.gpu_compare)
  {
    int repeat = (opt.repeat > 0) ? opt.repeat : 1;
    const auto query_start = TimingClock::now();

    if (opt.use_gpu)
    {
      query.setQueryFlags(query.queryFlags() | ohm::kQfGpuEvaluate);
    }

    for (int i = 0; i < repeat; ++i)
    {
      query.reset(opt.hard_reset_on_repeat);
      query.execute();
    }
    const auto query_end = TimingClock::now();

    showTiming(query_name, query_start, query_end, repeat);
  }
  else
  {
    compareCpuGpuQuery(query_name, query, range_epsilon);
  }
}


int runQueries(const Options &opt)
{
  ohm::OccupancyMap map(1);
  ProgressMonitor prog(10);

  if (!opt.haveQuery())
  {
    printf("Nothing to do! Queries not specified.\n");
    return 1;
  }

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress & prog)
  {
    if (!opt.quiet)
    {
      if (prog.info.total)
      {
        printf("\r%7" PRIu64 " / %7" PRIu64 "    ", prog.progress, prog.info.total);
      }
      else
      {
        printf("\r%7" PRIu64 "    ", prog.progress);
      }
      fflush(stdout);
    }
  });
  // Start paused.
  prog.startThread(true);

  printf("Loading map %s\n", opt.map_file.c_str());
  LoadMapProgress load_progress(prog);
  prog.unpause();
  int err = ohm::load(opt.map_file.c_str(), map, nullptr);//&loadProgress);
  prog.endProgress();

  if (err)
  {
    printf("Failed to load map %s ", opt.map_file.c_str());
    switch (err)
    {
    case ohm::kSeFileCreateFailure:
      printf("file create failure\n");
      break;
    case ohm::kSeFileOpenFailure:
      printf("file open failure\n");
      break;
    case ohm::kSeFileWriteFailure:
      printf("file write failure\n");
      break;
    case ohm::kSeFileReadFailure:
      printf("file read failure\n");
      break;
    case ohm::kSeValueOverflow:
      printf("value overflow\n");
      break;
    case ohm::kSeUnsupportedVersion:
      printf("unsupported version\n");
      break;
    default:
      printf("unknown (%d)\n", err);
      break;
    }
    return 2;
  }

  //timing_clock::time_point queryStart, queryEnd;
  std::string str;
  unsigned query_flags = 0;
  query_flags |= !!opt.unknown_as_occupied * ohm::kQfUnknownAsOccupied;
  // queryFlags |= ohm::kQfGpuEvaluate;

  if (opt.neighbours.radius >= 0)
  {
    std::cout << "Running nearest neighbours " << opt.neighbours.point << " R: " << opt.neighbours.radius << std::endl;

    ohm::NearestNeighbours nn_query(map, opt.neighbours.point,
                                   opt.neighbours.radius, query_flags);
    executeQuery("nearest neighbours", opt, nn_query);
    saveQueryCloud(map, nn_query, opt, "-near");
  }

  if (opt.line.radius >= 0)
  {
    printf("Running line query (%lg %lg %lg)->(%lg %lg %lg) R: %lg\n",
           opt.line.start.x, opt.line.start.y, opt.line.start.z,
           opt.line.end.x, opt.line.end.y, opt.line.end.z,
           opt.line.radius
          );

    ohm::LineQuery line_query(map, opt.line.start, opt.line.end, opt.line.radius, query_flags);
    // Allow single voxel epsilon value.
    executeQuery("line query", opt, line_query, float(map.resolution()));
    saveQueryCloud(map, line_query, opt, "-line", opt.line.radius);
  }

#ifdef FIXME
  if (opt.ranges.radius > 0)
  {
    printf("Running ranges query (%lg %lg %lg)->(%lg %lg %lg) R: %lg\n",
           opt.ranges.min.x, opt.ranges.min.y, opt.ranges.min.z,
           opt.ranges.max.x, opt.ranges.max.y, opt.ranges.max.z,
           opt.ranges.radius
          );

    ohm::VoxelRanges rangesQuery(map, opt.ranges.min, opt.ranges.max, opt.ranges.radius, queryFlags);
    auto queryStart = timing_clock::now();
    if (opt.gpuCompare)
    {
      rangesQuery.setQueryFlags(rangesQuery.queryFlags() & ~ohm::kQfGpuEvaluate);
      printf("CPU: ");
      fflush(stdout);
    }
    else if (opt.useGpu)
    {
      rangesQuery.setQueryFlags(rangesQuery.queryFlags() | ohm::kQfGpuEvaluate);
    }

    rangesQuery.reset();
    rangesQuery.execute();
    auto queryEnd = timing_clock::now();

    showTiming("ranges query", queryStart, queryEnd, 1);

    if (opt.gpuCompare)
    {
      std::vector<glm::vec4> gpuPoints, cpuPoints;
      // Extract CPU points.
      saveRangesCloud(map, rangesQuery, "-ranges-cpu", prog, opt, &cpuPoints, true);
      // Set GPU flag and run again on GPU.
      queryStart = timing_clock::now();
      rangesQuery.setQueryFlags(rangesQuery.queryFlags() | ohm::kQfGpuEvaluate);

      printf("GPU: ");
      fflush(stdout);
      rangesQuery.reset();
      rangesQuery.execute();
      queryEnd = timing_clock::now();
      showTiming("ranges query", queryStart, queryEnd, 1);

      // Extract CPU points
      saveRangesCloud(map, rangesQuery, "-ranges-gpu", prog, opt, &gpuPoints);

      // Compare.
      size_t pointLimit = cpuPoints.size();
      if (cpuPoints.size() != gpuPoints.size())
      {
        std::cout << "Point count difference. CPU: " << cpuPoints.size() << " GPU: " << gpuPoints.size() << std::endl;
        pointLimit = std::min(cpuPoints.size(), gpuPoints.size());
      }

      size_t coordMismatch = 0;
      size_t rangeMismatch = 0;
      float distSqr = 0;
      const float epsilon = 1e-4f;
      const float rangeEpsilon = float(0.3 * map.resolution());
      glm::vec3 separation;

      std::cout << "Comparing with allowed obstacle range epsilon of " << rangeEpsilon << std::endl;
      if (cpuPoints.size() == gpuPoints.size())
      {
        for (size_t i = 0; i < pointLimit; ++i)
        {
          separation = glm::vec3(cpuPoints[i]) - glm::vec3(gpuPoints[i]);
          distSqr = glm::dot(separation, separation);
          if (distSqr > epsilon)
          {
            ++coordMismatch;
          }
          // Only check range if coordinates are ok. We expect that if the coordinates don't
          // match, then neither do the ranges.
          else if (std::abs(cpuPoints[i].w - gpuPoints[i].w) > rangeEpsilon)
          {
            ++rangeMismatch;
            // Let's have a look at the generating voxel.
            ohm::Key key = map.voxelKey(glm::vec3(cpuPoints[i]));
            ohm::VoxelConst voxel = map.voxel(key);
            printf("  Range mismatch @ (%f %f %f) %f != %f : Node type %s\n",
                   cpuPoints[i].x, cpuPoints[i].y, cpuPoints[i].z,
                   cpuPoints[i].w, gpuPoints[i].w,
                   ohm::occupancyTypeToString(map.occupancyType(voxel))
                  );
          }
        }
      }
      else
      {
        for (size_t i = 0; i < gpuPoints.size(); ++i)
        {
          bool foundMatch = false;
          for (size_t j = 0; j < cpuPoints.size(); ++j)
          {
            separation = glm::vec3(cpuPoints[j]) - glm::vec3(gpuPoints[i]);
            distSqr = glm::dot(separation, separation);
            if (distSqr <= epsilon)
            {
              foundMatch = true;
              // Only check range if coordinates are ok. We expect that if the coordinates don't
              // match, then neither do the ranges.
              if (std::abs(cpuPoints[j].w - gpuPoints[i].w) > rangeEpsilon)
              {
                ++rangeMismatch;
                // Let's have a look at the generating voxel.
                ohm::Key key = map.voxelKey(glm::vec3(gpuPoints[j]));
                ohm::VoxelConst voxel = map.voxel(key);
                printf("  Range mismatch @ (%f %f %f) : %f != %f. Node [%d %d %d] %s\n",
                       gpuPoints[i].x, gpuPoints[i].y, gpuPoints[i].z,
                       gpuPoints[i].w, cpuPoints[j].w,
                       int(key.localKey().x), int(key.localKey().y), int(key.localKey().z),
                       ohm::occupancyTypeToString(map.occupancyType(voxel))
                      );
              }
              break;
            }
          }

          if (!foundMatch)
          {
            ++coordMismatch;
          }
        }
      }

      if (coordMismatch || rangeMismatch)
      {
        std::cerr << "Comparison failure. " << coordMismatch << " coordinate mismatch(es), "
                  << rangeMismatch << " range mismatch(es) of " << pointLimit << " points." << std::endl;
      }
    }
    else
    {
      saveRangesCloud(map, rangesQuery, "-ranges", prog, opt);
    }
  }
#endif // FIXME

  return 0;
}

int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  // Generate output name based on input if not specified.
  if (opt.output_base.empty())
  {
    const auto extension_start = opt.map_file.find_last_of(".");
    if (extension_start != std::string::npos)
    {
      opt.output_base = opt.map_file.substr(0, extension_start);
    }
    else
    {
      opt.output_base = opt.map_file;
    }
  }

  opt.print();

  // Initialise TES
  TES_SETTINGS(settings, tes::SF_Compress | tes::SF_Collate);
  // Initialise server info.
  TES_SERVER_INFO(info, tes::XYZ);
  // Create the server. Use tesServer declared globally above.
  TES_SERVER_CREATE(ohm::g_3es, settings, &info);
  TES_STMT(ohm::g_3es = ohm::g_3es);

  // Start the server and wait for the connection monitor to start.
  TES_SERVER_START(ohm::g_3es, tes::ConnectionMonitor::Asynchronous);
  TES_SERVER_START_WAIT(ohm::g_3es, 1000);

#ifdef TES_ENABLE
  std::cout << "Starting with " << ohm::g_3es->connectionCount() << " connection(s)." << std::endl;
#endif // TES_ENABLE

  initialiseDebugCategories(opt);

  if (opt.use_gpu)
  {
    res = ohm::configureGpuFromArgs(argc, argv);
    if (res)
    {
      std::cerr << "Failed to configure GPU." << std::endl;
      return res;
    }
  }

  res = runQueries(opt);
  TES_SERVER_STOP(ohm::g_3es);
  return res;
}
