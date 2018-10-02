//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/mapcache.h>
#include <ohm/occupancykey.h>
#include <ohm/occupancykeylist.h>
#include <ohm/occupancymap.h>
#include <ohm/occupancymapserialise.h>
#include <ohm/occupancynode.h>
#include <ohm/occupancytype.h>
#include <ohmutil/plymesh.h>
#include <ohmutil/progressmonitor.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_set>

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

  enum ExportMode
  {
    ExportOccupancy,
    ExportClearance
  };

  struct Options
  {
    std::string mapFile;
    std::string plyFile;
    // expire regions older than this
    double expiryTime = 0;
    float cullDistance = 0;
    float occupancyThreshold = -1.0f;
    float colourScale = 3.0f;
    ExportMode mode = ExportOccupancy;
  };

  template <typename NUMERIC> bool optionValue(const char *arg, int argc, char *argv[], NUMERIC &value)
  {
    std::string strValue;
    if (optionValue(arg, argc, argv, strValue))
    {
      std::istringstream instr(strValue);
      instr >> value;
      return !instr.fail();
    }

    return false;
  }

  class LoadMapProgress : public ohm::SerialiseProgress
  {
  public:
    LoadMapProgress(ProgressMonitor &monitor)
      : _monitor(monitor)
    {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { _monitor.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc = 1) override { _monitor.incrementProgressBy(inc); }

  private:
    ProgressMonitor &_monitor;
  };
}


// Custom option parsing. Must come before we include options.h/cxxopt.hpp
std::istream &operator>>(std::istream &in, ExportMode &mode)
{
  std::string modeStr;
  in >> modeStr;
  if (modeStr.compare("occupancy") == 0)
  {
    mode = ExportOccupancy;
  }
  else if (modeStr.compare("clearance") == 0)
  {
    mode = ExportClearance;
  }
  // else
  // {
  //   throw cxxopts::invalid_option_format_error(modeStr);
  // }
  return in;
}

std::ostream &operator<<(std::ostream &out, const ExportMode mode)
{
  switch (mode)
  {
  case ExportOccupancy:
    out << "occupancy";
    break;
  case ExportClearance:
    out << "clearance";
    break;
  }
  return out;
}


// Must be after argument streaming operators.
#include <ohmutil/options.h>

int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options optParse(argv[0], "Convert an occupancy map to a point cloud. Defaults to generate a positional "
                                     "point cloud, but can generate a clearance cloud as well.");
  optParse.positional_help("<map.ohm> <cloud.ply>");

  try
  {
    // Build GPU options set.
    // clang-format off
    optParse.add_options()
      ("help", "Show help.")
      ("colour-scale", "Colour max scaling value for colouring a clearance cloud. Max colour at this range..", cxxopts::value(opt.colourScale))
      ("cloud", "The output cloud file (ply).", cxxopts::value(opt.plyFile))
      ("cull", "Remove regions farther than the specified distance from the map origin.", cxxopts::value(opt.cullDistance)->default_value(optStr(opt.cullDistance)))
      ("map", "The input map file (ohm).", cxxopts::value(opt.mapFile))
      ("mode", "Export mode [occupancy,clearance]: select which data to export from the map.", cxxopts::value(opt.mode)->default_value(optStr(opt.mode)))
      ("expire", "Expire regions with a timestamp before the specified time. These are not exported.", cxxopts::value(opt.expiryTime))
      ("threshold", "Override the map's occupancy threshold. Only occupied points are exported.", cxxopts::value(opt.occupancyThreshold)->default_value(optStr(opt.occupancyThreshold)))
      ;
    // clang-format on

    optParse.parse_positional({ "map", "cloud" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Map", "Mapping", "GPU" }) << std::endl;
      return 1;
    }

    if (opt.mapFile.empty())
    {
      std::cerr << "Missing input map file name" << std::endl;
      return -1;
    }
    if (opt.plyFile.empty())
    {
      std::cerr << "Missing output file name" << std::endl;
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

  std::cout << "Loading map " << opt.mapFile.c_str() << std::endl;
  ProgressMonitor prog(10);
  LoadMapProgress loadProgress(prog);
  ohm::OccupancyMap map(1.0f);

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog) {
    // if (!opt.quiet)
    {
      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (prog.info.info && prog.info.info[0])
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
  res = ohm::load(opt.mapFile.c_str(), map, &loadProgress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error code: " << res << std::endl;
    return res;
  }

  if (opt.occupancyThreshold >= 0)
  {
    map.setOccupancyThresholdProbability(opt.occupancyThreshold);
  }

  if (opt.cullDistance)
  {
    std::cout << "Culling regions beyond range : " << opt.cullDistance << std::endl;
    const unsigned removed = map.removeDistanceRegions(map.origin(), opt.cullDistance);
    std::cout << "Removed " << removed << " regions" << std::endl;
    ;
  }
  if (opt.expiryTime)
  {
    std::cout << "Expiring regions before time: " << opt.expiryTime << std::endl;
    unsigned removed = map.expireRegions(opt.expiryTime);
    std::cout << "Removed " << removed << " regions" << std::endl;
  }

  std::cout << "Converting to PLY cloud" << std::endl;
  PlyMesh ply;
  glm::vec3 v;
  size_t regionCount = map.regionCount();
  glm::i16vec3 lastRegion = map.begin().key().regionKey();
  uint64_t pointCount = 0;

  prog.beginProgress(ProgressMonitor::Info(regionCount));

  for (auto iter = map.begin(); iter != map.end() && !quit; ++iter)
  {
    const ohm::OccupancyNodeConst node = *iter;
    if (lastRegion != iter.key().regionKey())
    {
      prog.incrementProgress();
      lastRegion = iter.key().regionKey();
    }
    if (opt.mode == ExportOccupancy)
    {
      if (map.occupancyType(node) == ohm::Occupied)
      {
        v = map.voxelCentreLocal(node.key());
        ply.addVertex(v);
        ++pointCount;
      }
    }
    else if (opt.mode == ExportClearance)
    {
      if (node.isValid() && node.clearance() >= 0 && node.clearance() < opt.colourScale)
      {
        const float rangeValue = node.clearance();
        uint8_t c = (uint8_t)(255 * std::max(0.0f, (opt.colourScale - rangeValue) / opt.colourScale));
        v = map.voxelCentreLocal(node.key());
        ply.addVertex(v, Colour(c, 128, 0));
        ++pointCount;
      }
    }
  }

  prog.endProgress();
  prog.pause();
  prog.joinThread();
  std::cout << "\nExporting " << pointCount << " points" << std::endl;

  if (!quit)
  {
    ply.save(opt.plyFile.c_str(), true);
  }

  return res;
}
