//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/HeightmapVoxel.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/MapCache.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/Voxel.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>

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
    kExportOccupancy,
    kExportOccupancyCentre,
    kExportClearance,
    kExportHeightmap,
  };

  struct Options
  {
    std::string map_file;
    std::string ply_file;
    // expire regions older than this
    double expiry_time = 0;
    float cull_distance = 0;
    float occupancy_threshold = -1.0f;
    float colour_scale = 3.0f;
    ExportMode mode = kExportOccupancy;
    std::string heightmap_axis = "z";
  };

  template <typename NUMERIC>
  bool optionValue(const char *arg, int argc, char *argv[], NUMERIC &value)
  {
    std::string str_value;
    if (optionValue(arg, argc, argv, str_value))
    {
      std::istringstream instr(str_value);
      instr >> value;
      return !instr.fail();
    }

    return false;
  }

  class LoadMapProgress : public ohm::SerialiseProgress
  {
  public:
    LoadMapProgress(ProgressMonitor &monitor)
      : monitor_(monitor)
    {}

    bool quit() const override { return ::quit > 1; }

    void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
    void incrementProgress(unsigned inc = 1) override { monitor_.incrementProgressBy(inc); }

  private:
    ProgressMonitor &monitor_;
  };
}  // namespace


// Custom option parsing. Must come before we include Options.h/cxxopt.hpp
std::istream &operator>>(std::istream &in, ExportMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  if (mode_str.compare("occupancy") == 0)
  {
    mode = kExportOccupancy;
  }
  else if (mode_str.compare("occupancy-centre") == 0)
  {
    mode = kExportOccupancyCentre;
  }
  else if (mode_str.compare("clearance") == 0)
  {
    mode = kExportClearance;
  }
  else if (mode_str.compare("heightmap") == 0)
  {
    mode = kExportHeightmap;
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
  case kExportOccupancy:
    out << "occupancy";
    break;
  case kExportOccupancyCentre:
    out << "occupancy-centre";
    break;
  case kExportClearance:
    out << "clearance";
    break;
  case kExportHeightmap:
    out << "heightmap";
    break;
  }
  return out;
}


// Must be after argument streaming operators.
#include <ohmutil/Options.h>

int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0], "Convert an occupancy map to a point cloud. Defaults to generate a positional "
                                      "point cloud, but can generate a clearance cloud as well.");
  opt_parse.positional_help("<map.ohm> <cloud.ply>");

  try
  {
    // Build GPU options set.
    // clang-format off
    opt_parse.add_options()
      ("help", "Show help.")
      ("colour-scale", "Colour max scaling value for colouring a clearance or heightmap cloud. Max colour at this range..", cxxopts::value(opt.colour_scale))
      ("cloud", "The output cloud file (ply).", cxxopts::value(opt.ply_file))
      ("cull", "Remove regions farther than the specified distance from the map origin.", cxxopts::value(opt.cull_distance)->default_value(optStr(opt.cull_distance)))
      ("map", "The input map file (ohm).", cxxopts::value(opt.map_file))
      ("mode", "Export mode [occupancy,occupancy-centre,clearance,heightmap]: select which data to export from the "
               "map. occupancy and occupancy-centre differ only in that the latter forces positioning on voxel "
               "centres.", cxxopts::value(opt.mode)->default_value(optStr(opt.mode)))
      ("heightmap-axis", "Axis for the heightmap vertical axis [x, y, z].", cxxopts::value(opt.heightmap_axis)->default_value(optStr(opt.heightmap_axis)))
      ("expire", "Expire regions with a timestamp before the specified time. These are not exported.", cxxopts::value(opt.expiry_time))
      ("threshold", "Override the map's occupancy threshold. Only occupied points are exported.", cxxopts::value(opt.occupancy_threshold)->default_value(optStr(opt.occupancy_threshold)))
      ;
    // clang-format on

    opt_parse.parse_positional({ "map", "cloud" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Map", "Mapping", "GPU" }) << std::endl;
      return 1;
    }

    if (opt.map_file.empty())
    {
      std::cerr << "Missing input map file name" << std::endl;
      return -1;
    }
    if (opt.ply_file.empty())
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

  int res = parseOptions(opt, argc, argv);

  if (res)
  {
    return res;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  std::cout << "Loading map " << opt.map_file.c_str() << std::endl;
  std::cout << "Export mode: " << opt.mode << std::endl;
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::OccupancyMap map(1.0f);

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog) {
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
  res = ohm::load(opt.map_file.c_str(), map, &load_progress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  int heightmap_axis = -1;
  if (opt.heightmap_axis.compare("x") == 0)
  {
    heightmap_axis = 0;
  }
  else if (opt.heightmap_axis.compare("y") == 0)
  {
    heightmap_axis = 1;
  }
  else if (opt.heightmap_axis.compare("z") == 0)
  {
    heightmap_axis = 2;
  }
  else
  {
    std::cerr << "Invalid heightmap axis: " << opt.heightmap_axis << std::endl;
    return -1;
  }

  // Validate the required layer is present.
  switch (opt.mode)
  {
  case kExportOccupancy:
  case kExportOccupancyCentre:
    if (map.layout().layer("occupancy") == nullptr)
    {
      std::cerr << "Missing 'occupancy' layer" << std::endl;
      return -1;
    }
    break;
  case kExportClearance:
    if (map.layout().layer("clearance") == nullptr)
    {
      std::cerr << "Missing 'clearance' layer" << std::endl;
      return -1;
    }
    break;
  case kExportHeightmap:
  {
    const ohm::MapLayer *layer = map.layout().layer(ohm::HeightmapVoxel::kHeightmapLayer);
    if (!layer)
    {
      std::cerr << "Missing '" << ohm::HeightmapVoxel::kHeightmapLayer << "' layer" << std::endl;
      return -1;
    }
    if (layer->voxelByteSize() < sizeof(ohm::HeightmapVoxel))
    {
      std::cerr << "Layer '" << ohm::HeightmapVoxel::kHeightmapLayer << "' is not large enough. Expect "
                << sizeof(ohm::HeightmapVoxel) << " actual " << layer->voxelByteSize() << std::endl;
      return -1;
    }

    break;
  }
  default:
    break;
  }

  if (opt.occupancy_threshold >= 0)
  {
    map.setOccupancyThresholdProbability(opt.occupancy_threshold);
  }

  if (opt.cull_distance)
  {
    std::cout << "Culling regions beyond range : " << opt.cull_distance << std::endl;
    const unsigned removed = map.removeDistanceRegions(map.origin(), opt.cull_distance);
    std::cout << "Removed " << removed << " regions" << std::endl;
    ;
  }
  if (opt.expiry_time)
  {
    std::cout << "Expiring regions before time: " << opt.expiry_time << std::endl;
    unsigned removed = map.expireRegions(opt.expiry_time);
    std::cout << "Removed " << removed << " regions" << std::endl;
  }

  std::cout << "Converting to PLY cloud" << std::endl;
  PlyMesh ply;
  glm::vec3 v;
  const size_t region_count = map.regionCount();
  glm::i16vec3 last_region = map.begin().key().regionKey();
  uint64_t point_count = 0;

  prog.beginProgress(ProgressMonitor::Info(region_count));

  for (auto iter = map.begin(); iter != map.end() && !quit; ++iter)
  {
    const ohm::VoxelConst voxel = *iter;
    if (last_region != iter.key().regionKey())
    {
      prog.incrementProgress();
      last_region = iter.key().regionKey();
    }
    if (opt.mode == kExportOccupancy || kExportOccupancyCentre)
    {
      if (map.occupancyType(voxel) == ohm::Occupied)
      {
        v = (opt.mode == kExportOccupancy) ? voxel.position() : voxel.centreGlobal();
        ply.addVertex(v);
        ++point_count;
      }
    }
    else if (opt.mode == kExportClearance)
    {
      if (voxel.isValid() && voxel.clearance() >= 0 && voxel.clearance() < opt.colour_scale)
      {
        const float range_value = voxel.clearance();
        uint8_t c = uint8_t(255 * std::max(0.0f, (opt.colour_scale - range_value) / opt.colour_scale));
        v = voxel.centreGlobal();
        ply.addVertex(v, Colour(c, 128, 0));
        ++point_count;
      }
    }
    else if (opt.mode == kExportHeightmap)
    {
      if (voxel.isValid() && voxel.isOccupied())
      {
        const ohm::HeightmapVoxel *voxel_height = voxel.layerContent<const ohm::HeightmapVoxel *>(
          map.layout().layer(ohm::HeightmapVoxel::kHeightmapLayer)->layerIndex());

        uint8_t c = uint8_t(255 * std::max(0.0f, (opt.colour_scale - voxel_height->clearance) / opt.colour_scale));
        if (voxel_height->clearance <= 0)
        {
          // Max clearance. No red.
          c = 0;
        }

        glm::dvec3 up(0);
        up[heightmap_axis] = 1;
        v = voxel.centreGlobal() + up * double(voxel_height->height);
        ply.addVertex(v, Colour(c, 128, 0));
        ++point_count;
      }
    }
  }

  prog.endProgress();
  prog.pause();
  prog.joinThread();
  std::cout << "\nExporting " << point_count << " points" << std::endl;

  if (!quit)
  {
    ply.save(opt.ply_file.c_str(), true);
  }

  return res;
}
