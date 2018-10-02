//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/mapcache.h>
#include <ohm/occupancykey.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancymapserialise.h>
#include <ohm/occupancynode.h>
#include <ohm/occupancytype.h>

#include <ohmutil/ohmutil.h>
#include <ohmutil/progressmonitor.h>
#include <ohmutil/options.h>

#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <unordered_set>

#define PROFILING 1
#include <ohmutil/profile.h>

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
    std::string mapFileIn;
    std::string cloudFileIn;
    std::string cloudFileOut;
    float occupancyThreshold = 0;
  };


  class MapProgress : public ohm::SerialiseProgress
  {
  public:
    MapProgress(ProgressMonitor &monitor) : _monitor(monitor) {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { _monitor.beginProgress(ProgressMonitor::Info("Loading", target)); }
    void incrementProgress(unsigned inc = 1) override { _monitor.incrementProgressBy(inc); }

  private:
    ProgressMonitor &_monitor;
  };
}


int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options optParse(argv[0],
    "\nFilters a cloud against an occupancy map, removing points in free voxels."
  );
  optParse.positional_help("<map-in.ohm> <cloud-in.laz> <cloud-out.laz>");

  try
  {
    // clang-format off
    optParse.add_options()
        ("cloud-in", "Input cloud to filter", cxxopts::value(opt.cloudFileIn))
        ("cloud-out", "Clout to write filtered results to", cxxopts::value(opt.cloudFileOut))
        ("map", "Input occupancy map", cxxopts::value(opt.mapFileIn))
        ("threshold", "Changes the occupancy probability threshold from that used to generate the map.", optVal(opt.occupancyThreshold), "[0, 1]")
      ;
    // clang-format on

    optParse.parse_positional({ "map", "cloud-in", "cloud-out" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt.mapFileIn.empty())
    {
      std::cerr << "Missing input map" << std::endl;
      return -1;
    }
    if (opt.cloudFileIn.empty())
    {
      std::cerr << "Missing input cloud" << std::endl;
      return -1;
    }
    if (opt.cloudFileOut.empty())
    {
      std::cerr << "Missing output cloud" << std::endl;
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


size_t filterCloud(ProgressMonitor &prog, const ohm::OccupancyMap &map, const Options &opt)
{
  size_t exported = 0;
  size_t numberOfPoints = 0;

  std::ifstream ifs;
  std::ofstream ofs;

  ifs.open(opt.cloudFileIn.c_str(), std::ios::in | std::ios::binary);
  ofs.open(opt.cloudFileOut.c_str(), std::ios::out | std::ios::binary);

  bool ok = true;

  if (!ifs.is_open())
  {
    ok = false;
    std::cerr << "Unable to read " << opt.cloudFileIn << std::endl;
  }

  if (!ofs.is_open())
  {
    ok = false;
    std::cerr << "Unable to write " << opt.cloudFileOut << std::endl;
  }

  if (!ok)
  {
    return false;
  }

  std::string extensionStr = opt.cloudFileOut;
  bool compress = false;

  if (extensionStr.size() >= 4)
  {
    extensionStr = extensionStr.substr(extensionStr.size() - 4);
    std::transform(extensionStr.begin(), extensionStr.end(), extensionStr.begin(), &tolower);
    if (extensionStr.compare(".laz") == 0)
    {
      compress = true;
    }
  }

  liblas::ReaderFactory readerFactor;
  try
  {
    liblas::Reader reader(readerFactor.CreateWithStream(ifs));
    liblas::Header header = reader.GetHeader();
    header.SetCompressed(compress);
    liblas::Writer writer(ofs, header);
    numberOfPoints = reader.GetHeader().GetPointRecordsCount();

    prog.beginProgress(ProgressMonitor::Info("Filtering", reader.GetHeader().GetPointRecordsCount()));

    ohm::OccupancyNodeConst node;
    ohm::MapCache cache;
    glm::dvec3 pos;
    while (reader.ReadNextPoint())
    {
      // Get the voxel for the point.
      auto &&pt = reader.GetPoint();
      pos = glm::dvec3(pt.GetX(), pt.GetY(), pt.GetZ());
      node = map.node(map.voxelKey(pos), &cache);
      if (node.isOccupied())
      {
        writer.WritePoint(reader.GetPoint());
        ++exported;
      }
      prog.incrementProgress();
    }
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  prog.endProgress();
  std::cout << std::endl;

  std::cout << "Exported " << exported << " / " <<  numberOfPoints << " point(s)" << std::endl;

  return exported;
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

  printf("Loading map %s\n", opt.mapFileIn.c_str());
  ProgressMonitor prog(10);
  MapProgress loadProgress(prog);
  ohm::OccupancyMap map(1.0f);

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog)
  {
    //if (!opt.quiet)
    {
      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (prog.info.info)
      {
        out << prog.info.info << " : ";
      }

      out << std::setfill(' ') << std::setw(12) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(12) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });

  prog.startThread();
  res = ohm::load(opt.mapFileIn.c_str(), map, &loadProgress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    fprintf(stderr, "Failed to load map. Error code: %d\n", res);
    return res;
  }

  printf("\n");

  if (opt.occupancyThreshold > 0)
  {
    map.setOccupancyThresholdProbability(opt.occupancyThreshold);
  }

  // Start filtering the cloud.
  filterCloud(prog, map, opt);

  prog.joinThread();

  return res;
}
