//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelLayout.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/Options.h>

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
    std::string map_file;
  };
}


int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options optParse(argv[0],
"\nProvide information about the contents of an occupancy map file.\n"
);
  optParse.positional_help("<map.ohm>");

  try
  {
    optParse.add_options()
      ("help", "Show help.")
      ("i,map", "The input map file (ohm) to load.", cxxopts::value(opt.map_file))
      ;

    optParse.parse_positional({ "map" });

    cxxopts::ParseResult parsed = optParse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << optParse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt.map_file.empty())
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

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  printf("Loading map %s\n", opt.map_file.c_str());
  ohm::OccupancyMap map(1.0f);
  ohm::MapVersion version;

  res = ohm::loadHeader(opt.map_file.c_str(), map, &version);

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

  std::cout << "]" << std::endl;

  // Data needing chunks to be partly loaded.
  // - Extents
  // - Region count
  // - Memory footprint

  const ohm::MapLayout &layout = map.layout();
  std::string indent;
  std::string vox_size_str, region_size_str;
  std::cout << "Layers: " << layout.layerCount() << std::endl;

  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    const ohm::MapLayer &layer = layout.layer(i);
    ohm::VoxelLayoutConst voxels = layer.voxelLayout();
    indent = "  ";
    std::cout << indent << layer.name() << std::endl;
    std::cout << indent << "subsampling: " << layer.subsampling() << std::endl;
    std::cout << indent << "voxels: " << layer.dimensions(map.regionVoxelDimensions()) << " : " << layer.volume(layer.dimensions(map.regionVoxelDimensions())) << std::endl;

    util::makeMemoryDisplayString(vox_size_str, voxels.voxelByteSize());
    std::cout << indent << "voxel byte size: " << vox_size_str << std::endl;
    util::makeMemoryDisplayString(region_size_str, voxels.voxelByteSize() * layer.volume(layer.dimensions(map.regionVoxelDimensions())));
    std::cout << indent << "region byte size: " << region_size_str << std::endl;

    indent += "  ";
    std::cout << std::setw(4) << std::setfill('0');
    for (size_t i = 0; i < voxels.memberCount(); ++i)
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
