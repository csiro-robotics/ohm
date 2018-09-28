//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include "maplayout.h"
#include "occupancymap.h"
#include "occupancymapserialise.h"
#include "occupancyutil.h"
#include "ohmvoxellayout.h"

#include <ohmutil.h>
#include <options.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
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

  struct Options
  {
    std::string mapFile;
  };


  // class LoadMapProgress : public ohm::SerialiseProgress
  // {
  // public:
  //   LoadMapProgress(ProgressMonitor &monitor) : _monitor(monitor) {}

  //   bool quit() const override { return ::quit > 1; }

  //   void setTargetProgress(unsigned target) override { _monitor.beginProgress(ProgressMonitor::Info(target)); }
  //   void incrementProgress(unsigned inc = 1) override { _monitor.incrementProgressBy(inc); }

  // private:
  //   ProgressMonitor &_monitor;
  // };
}


int parseOptions(Options &opt, int argc, const char *argv[])
{
  cxxopts::Options optParse(argv[0],
"\nProvide information about the contents of an occupancy map file.\n"
);
  optParse.positional_help("<map.ohm>");

  try
  {
    optParse.add_options()
      ("help", "Show help.")
      ("i,map", "The input map file (ohm) to load.", cxxopts::value(opt.mapFile))
      ;

    optParse.parse_positional({ "map" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt.mapFile.empty())
    {
      std::cerr << "Missing input map" << std::endl;
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

int main(int argc, const char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = 0;
  res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  printf("Loading map %s\n", opt.mapFile.c_str());
  // ProgressMonitor prog(10);
  // LoadMapProgress loadProgress(prog);
  ohm::OccupancyMap map(1.0f);
  ohm::MapVersion version;

  // prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog)
  // {
  //   std::ostringstream str;

  //   str << '\r';

  //   str << prog.progress;

  //   if (prog.info.total)
  //   {
  //     str << " / " << prog.info.total;
  //   }

  //   str << "      ";

  //   std::cout << str.str() << std::flush;
  // });

  // prog.startThread();
  res = ohm::loadHeader(opt.mapFile.c_str(), map, &version);
  // prog.endProgress();

  // std::cout << std::endl;;

  if (res != 0)
  {
    fprintf(stderr, "Failed to load map. Error code: %d\n", res);
    return res;
  }

  std::cout << "File format version: " << version.major << '.' << version.minor << '.' << version.patch << std::endl;
  std::cout << std::endl;

  std::cout << "Voxel resolution: " << map.resolution() << std::endl;
  std::cout << "Map origin: " << map.origin() << std::endl;
  std::cout << "Region spatial dimensions: " << map.regionSpatialResolution() << std::endl;
  std::cout << "Region voxel dimensions: " << map.regionVoxelDimensions() << " : " << map.regionVoxelVolume() << std::endl;
  std::cout << std::endl;

  std::cout << "Occupancy threshold: " << map.occupancyThresholdProbability() << " (" << map.occupancyThresholdValue() << ")" <<  std::endl;
  std::cout << "Hit Probability: " << map.hitProbability() << " (" << map.hitValue() << ")" << std::endl;
  std::cout << "Miss probability: " << map.missProbability() << " (" << map.missValue() << ")" <<  std::endl;
  std::cout << "Saturation min/max: [";
  {
    if (map.saturateAtMinValue())
    {
      std::cout << map.minNodeProbability();
    }
    else
    {
      std::cout << "off";
    }
    std::cout << ",";
    if (map.saturateAtMaxValue())
    {
      std::cout << map.maxNodeProbability();
    }
    else
    {
      std::cout << "off";
    }
  }

  std::cout << std::endl;

  // Data needing chunks to be partly loaded.
  // - Extents
  // - Region count
  // - Memory footprint

  const ohm::MapLayout &layout = map.layout();
  std::string indent;
  std::string voxSizeStr, regionSizeStr;
  std::cout << "Layers: " << layout.layerCount() << std::endl;

  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const ohm::MapLayer &layer = layout.layer(i);
    ohm::VoxelLayoutConst voxels = layer.voxelLayout();
    indent = "  ";
    std::cout << indent << layer.name() << std::endl;
    std::cout << indent << "subsampling: " << layer.subsampling() << std::endl;
    std::cout << indent << "voxels: " << layer.dimensions(map.regionVoxelDimensions()) << " : " << layer.volume(layer.dimensions(map.regionVoxelDimensions())) << std::endl;

    util::makeMemoryDisplayString(voxSizeStr, voxels.voxelByteSize());
    std::cout << indent << "voxel byte size: " << voxSizeStr << std::endl;
    util::makeMemoryDisplayString(regionSizeStr, voxels.voxelByteSize() * layer.volume(layer.dimensions(map.regionVoxelDimensions())));
    std::cout << indent << "region byte size: " << regionSizeStr << std::endl;

    indent += "  ";
    std::cout << std::setw(4) << std::setfill('0');
    for (size_t m = 0; i < voxels.memberCount(); ++i)
    {
      std::cout << indent << "0x" << std::hex << voxels.memberOffset(i) << " "
                << ohm::DataType::name(voxels.memberType(i)) << " " << voxels.memberName(i)
                << " (0x" << voxels.memberSize(i) << ")"
                << std::endl;
    }
    std::cout << std::setw(0) << std::dec;
  }

  return res;
}
