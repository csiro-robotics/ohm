//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/DefaultLayer.h>
#include <ohm/Heightmap.h>
#include <ohm/HeightmapMesh.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/Key.h>
#include <ohm/KeyList.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyType.h>
#include <ohm/VoxelData.h>

#include <ohmtools/OhmCloud.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>

// CovarianceVoxel.h must be included later due to GPU suppport for this file.
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include <ohm/CovarianceVoxel.h>

#include <algorithm>
#include <array>
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
int g_quit = 0;

void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    ++g_quit;
  }
}

enum ExportMode
{
  kExportOccupancy,
  kExportOccupancyCentre,
  kExportDecay,
  kExportObserved,
  kExportClearance,
  kExportHeightmap,
  kExportHeightmapMesh,
  kExportCovariance
};

enum ColourMode
{
  kColourNone,
  kColourHeight,
  kColourLayer,
  kColourType
};

struct HeightmapOptions
{
  bool collapse = false;  ///< Collapse a layered heightmap into a 2.5D heightmap.
};

struct Options
{
  std::string map_file;
  std::string ply_file;
  // expire regions older than this
  double expiry_time = 0;
  float cull_distance = 0;
  float threshold = -1.0f;
  float colour_scale = 3.0f;
  ExportMode mode = kExportOccupancy;
  ColourMode colour = kColourHeight;

  HeightmapOptions heightmap;
};

template <typename NUMERIC>
bool optionValue(const char *arg, int argc, char *argv[], NUMERIC &value)  // NOLINT(modernize-avoid-c-arrays)
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
  explicit LoadMapProgress(ProgressMonitor &monitor)
    : monitor_(monitor)
  {}

  bool quit() const override { return ::g_quit > 1; }

  void setTargetProgress(unsigned target) override { monitor_.beginProgress(ProgressMonitor::Info(target)); }
  void incrementProgress(unsigned inc) override { monitor_.incrementProgressBy(inc); }

private:
  ProgressMonitor &monitor_;
};


/// Build a sphere approximation with an icosahedron.
/// @todo Make one subdivision for better spheres.
void makeUnitSphere(std::vector<glm::dvec3> &vertices, std::vector<unsigned> &indices)
{
  // We start with two hexagonal rings to approximate the sphere.
  // All subdivision occurs on a unit radius sphere, at the origin. We translate and
  // scale the vertices at the end.
  vertices.clear();
  indices.clear();

  const double ring_control_angle = 25.0 / 180.0 * M_PI;
  const double ring_height = std::sin(ring_control_angle);
  const double ring_radius = std::cos(ring_control_angle);
  const double hex_angle = 2.0 * M_PI / 6.0;
  const double ring2_offset_angle = 0.5 * hex_angle;
  const std::array<glm::dvec3, 14> initial_vertices =  // NOLINT(readability-magic-numbers)
    {
      glm::dvec3(0, 0, 1),

      // Upper hexagon.
      glm::dvec3(ring_radius, 0, ring_height),
      glm::dvec3(ring_radius * std::cos(hex_angle), ring_radius * std::sin(hex_angle), ring_height),
      glm::dvec3(ring_radius * std::cos(2 * hex_angle), ring_radius * std::sin(2 * hex_angle), ring_height),
      glm::dvec3(ring_radius * std::cos(3 * hex_angle), ring_radius * std::sin(3 * hex_angle), ring_height),
      glm::dvec3(ring_radius * std::cos(4 * hex_angle), ring_radius * std::sin(4 * hex_angle), ring_height),
      glm::dvec3(ring_radius * std::cos(5 * hex_angle), ring_radius * std::sin(5 * hex_angle), ring_height),

      // Lower hexagon.
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle), ring_radius * std::sin(ring2_offset_angle), -ring_height),
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle + hex_angle),
                 ring_radius * std::sin(ring2_offset_angle + hex_angle), -ring_height),
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle + 2 * hex_angle),
                 ring_radius * std::sin(ring2_offset_angle + 2 * hex_angle), -ring_height),
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle + 3 * hex_angle),
                 ring_radius * std::sin(ring2_offset_angle + 3 * hex_angle), -ring_height),
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle + 4 * hex_angle),
                 ring_radius * std::sin(ring2_offset_angle + 4 * hex_angle), -ring_height),
      glm::dvec3(ring_radius * std::cos(ring2_offset_angle + 5 * hex_angle),
                 ring_radius * std::sin(ring2_offset_angle + 5 * hex_angle), -ring_height),

      glm::dvec3(0, 0, -1),
    };

  const std::array<unsigned, 72> initial_indices =  // NOLINT(readability-magic-numbers)
    { 0,  1,  2, 0,  2,  3, 0, 3,  4, 0, 4,  5, 0, 5,  6,  0,  6,  1,  1,  7,  2,  2,  8,  3,
      3,  9,  4, 4,  10, 5, 5, 11, 6, 6, 12, 1, 7, 8,  2,  8,  9,  3,  9,  10, 4,  10, 11, 5,
      11, 12, 6, 12, 7,  1, 7, 13, 8, 8, 13, 9, 9, 13, 10, 10, 13, 11, 11, 13, 12, 12, 13, 7 };

  for (const auto vertex : initial_vertices)
  {
    vertices.emplace_back(vertex);
  }

  for (auto index : initial_indices)
  {
    indices.emplace_back(index);
  }
}
}  // namespace


// Custom option parsing. Must come before we include Options.h/cxxopt.hpp
void badArg(const std::string &arg);

std::istream &operator>>(std::istream &in, ExportMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "occupancy")
  {
    mode = kExportOccupancy;
  }
  else if (mode_str == "occupancy-centre")
  {
    mode = kExportOccupancyCentre;
  }
  else if (mode_str == "decay")
  {
    mode = kExportDecay;
  }
  else if (mode_str == "observed")
  {
    mode = kExportObserved;
  }
  else if (mode_str == "clearance")
  {
    mode = kExportClearance;
  }
  else if (mode_str == "heightmap")
  {
    mode = kExportHeightmap;
  }
  else if (mode_str == "heightmap-mesh")
  {
    mode = kExportHeightmapMesh;
  }
  else if (mode_str == "covariance")
  {
    mode = kExportCovariance;
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
  case kExportObserved:
    out << "observed";
    break;
  case kExportDecay:
    out << "decay";
    break;
  case kExportClearance:
    out << "clearance";
    break;
  case kExportHeightmap:
    out << "heightmap";
    break;
  case kExportHeightmapMesh:
    out << "heightmap-mesh";
    break;
  case kExportCovariance:
    out << "covariance";
    break;
  }
  return out;
}


std::istream &operator>>(std::istream &in, ColourMode &mode)
{
  std::string mode_str;
  in >> mode_str;
  if (mode_str == "none")
  {
    mode = kColourNone;
  }
  else if (mode_str == "height")
  {
    mode = kColourHeight;
  }
  else if (mode_str == "layer")
  {
    mode = kColourLayer;
  }
  else if (mode_str == "type")
  {
    mode = kColourType;
  }
  else
  {
    badArg(mode_str);
  }
  return in;
}

std::ostream &operator<<(std::ostream &out, const ColourMode mode)
{
  switch (mode)
  {
  case kColourNone:
    out << "none";
    break;
  case kColourHeight:
    out << "height";
    break;
  case kColourLayer:
    out << "layer";
    break;
  case kColourType:
    out << "type";
    break;
  default:
    out << "<unknown>";
  }
  return out;
}


// Must be after argument streaming operators.
#include <ohmutil/Options.h>

void badArg(const std::string &arg)
{
  // Must be declared before, but included after cxxopt.
  throw cxxopts::invalid_option_format_error(arg);
}


int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
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
      ("colour-scale", "Colour max scaling value for colouring a clearance or heightmap cloud. Max colour at this range..", cxxopts::value(opt->colour_scale))
      ("colour", "Colour control for exported point clouds {none,height,type}.", optVal(opt->colour))
      ("cloud", "The output cloud file (ply).", cxxopts::value(opt->ply_file))
      ("cull", "Remove regions farther than the specified distance from the map origin.", cxxopts::value(opt->cull_distance)->default_value(optStr(opt->cull_distance)))
      ("map", "The input map file (ohm).", cxxopts::value(opt->map_file))
      ("mode", "Export mode [occupancy,occupancy-centre,decay,clearance,covariance,heightmap,heightmap-mesh]: select "
               "which data to export from the map. occupancy and occupancy-centre differ only in that the latter "
               "forces positioning on voxel centres. decay uses the decay rate calculation and requires the map have "
               "traversability and voxel mean info - also uses threshold as a decay threshold instead of occupancy",
               cxxopts::value(opt->mode)->default_value(optStr(opt->mode)))
      ("expire", "Expire regions with a timestamp before the specified time. These are not exported.", cxxopts::value(opt->expiry_time))
      ("threshold", "Override the map's occupancy threshold or set the decay threshold. Only points passing the "
                    "threshold occupied points are exported.",
                    cxxopts::value(opt->threshold)->default_value(optStr(opt->threshold)))
      ;

    opt_parse.add_options("Heightmap")
      ("heightmap-2d", "Reduce to a 2.5D heightmap cloud regardless of input heightmap. This collapses a layered heightmap.", optVal(opt->heightmap.collapse))
      ;
    // clang-format on

    opt_parse.parse_positional({ "map", "cloud" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Heightmap" }) << std::endl;
      return 1;
    }

    if (opt->map_file.empty())
    {
      std::cerr << "Missing input map file name" << std::endl;
      return -1;
    }
    if (opt->ply_file.empty())
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


int exportPointCloud(const Options &opt, ProgressMonitor &prog, LoadMapProgress &load_progress)
{
  ohm::OccupancyMap map(1.0f);

  prog.startThread();
  int res = ohm::load(opt.map_file.c_str(), map, &load_progress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  // Validate the required layer is present.
  switch (opt.mode)
  {
  case kExportOccupancy:
  case kExportOccupancyCentre:
  case kExportObserved:
    if (map.layout().layer("occupancy") == nullptr)
    {
      std::cerr << "Missing 'occupancy' layer" << std::endl;
      res = -1;
    }
    break;
  case kExportClearance:
    if (map.layout().layer("clearance") == nullptr)
    {
      std::cerr << "Missing 'clearance' layer" << std::endl;
      res = -1;
    }
    break;
  case kExportDecay:
    if (map.layout().traversalLayer() == -1)
    {
      std::cout << "Missing 'traversal' layer" << std::endl;
      res = -1;
    }
    if (map.layout().meanLayer() == -1)
    {
      std::cout << "Missing 'mean' layer" << std::endl;
      res = -1;
    }
    break;
  case kExportHeightmap:
  {
    const ohm::MapLayer *layer = map.layout().layer(ohm::HeightmapVoxel::kHeightmapLayer);
    if (!layer)
    {
      std::cerr << "Missing '" << ohm::HeightmapVoxel::kHeightmapLayer << "' layer" << std::endl;
      res = -1;
    }
    else if (layer->voxelByteSize() < sizeof(ohm::HeightmapVoxel))
    {
      std::cerr << "Layer '" << ohm::HeightmapVoxel::kHeightmapLayer << "' is not large enough. Expect "
                << sizeof(ohm::HeightmapVoxel) << " actual " << layer->voxelByteSize() << std::endl;
      res = -1;
    }

    break;
  }
  default:
    std::cout << "Invalid mode for point cloud: " << opt.mode << std::endl;
    res = -1;
    break;
  }

  if (res != 0)
  {
    return res;
  }

  if (opt.threshold >= 0 && opt.mode != kExportDecay)
  {
    map.setOccupancyThresholdProbability(opt.threshold);
  }

  if (opt.cull_distance > 0)
  {
    std::cout << "Culling regions beyond range : " << opt.cull_distance << std::endl;
    const unsigned removed = map.removeDistanceRegions(map.origin(), opt.cull_distance);
    std::cout << "Removed " << removed << " regions" << std::endl;
    ;
  }
  if (opt.expiry_time > 0)
  {
    std::cout << "Expiring regions before time: " << opt.expiry_time << std::endl;
    unsigned removed = map.expireRegions(opt.expiry_time);
    std::cout << "Removed " << removed << " regions" << std::endl;
  }

  std::cout << "Converting to PLY cloud" << std::endl;
  const size_t region_count = map.regionCount();
  // uint64_t point_count = 0;

  prog.beginProgress(ProgressMonitor::Info(region_count));

  const auto save_progress_callback = [&prog](size_t progress, size_t /*target*/) { prog.updateProgress(progress); };
  ohmtools::ColourByHeight colour_by_height(map);
  ohmtools::ColourByType colour_by_type(map);
  ohmtools::ColourHeightmapType colour_by_heightmap_type(map);
  ohmtools::ColourHeightmapLayer colour_by_heightmap_layer(map);

  switch (opt.mode)
  {
  case kExportOccupancy:
    // fallthrough
  case kExportOccupancyCentre:
  case kExportObserved:
  {
    ohmtools::SaveCloudOptions save_opt;
    save_opt.ignore_voxel_mean = opt.mode != kExportOccupancy;
    save_opt.export_free = opt.mode == kExportObserved;
    // Default colour mode for saveCloud() is colour by height.
    save_opt.allow_default_colour_selection = (opt.colour == kColourHeight);
    if (opt.colour == kColourType)
    {
      save_opt.colour_select = [&colour_by_type](const ohm::Voxel<const float> &occupancy) {
        return colour_by_type.select(occupancy);
      };
    }
    saveCloud(opt.ply_file.c_str(), map, save_opt, save_progress_callback);
    break;
  }
  case kExportDecay:
  {
    ohmtools::SaveDecayCloudOptions save_opt;
    save_opt.ignore_voxel_mean = false;
    save_opt.allow_default_colour_selection = true;
    save_opt.decay_rate_threshold = opt.threshold;
    saveDecayCloud(opt.ply_file.c_str(), map, save_opt, save_progress_callback);
    break;
  }
  case kExportHeightmap:
  {
    ohmtools::SaveHeightmapCloudOptions save_opt;
    save_opt.ignore_voxel_mean = false;
    save_opt.export_free = true;
    save_opt.collapse = opt.heightmap.collapse;
    if (opt.colour == kColourHeight)
    {
      save_opt.colour_select = [&colour_by_height](const ohm::Voxel<const float> &occupancy) {
        return colour_by_height.select(occupancy);
      };
    }
    else if (opt.colour == kColourLayer)
    {
      save_opt.colour_select = [&colour_by_heightmap_layer](const ohm::Voxel<const float> &occupancy) {
        return colour_by_heightmap_layer.select(occupancy);
      };
    }
    else if (opt.colour == kColourType)
    {
      save_opt.colour_select = [&colour_by_heightmap_type](const ohm::Voxel<const float> &occupancy) {
        return colour_by_heightmap_type.select(occupancy);
      };
    }
    ohmtools::saveHeightmapCloud(opt.ply_file.c_str(), map, save_opt, save_progress_callback);
    break;
  }
  case kExportClearance:
  {
    glm::dvec3 min_ext;
    glm::dvec3 max_ext;
    map.calculateExtents(&min_ext, &max_ext);
    ohmtools::saveClearanceCloud(opt.ply_file.c_str(), map, min_ext, max_ext, opt.colour_scale, ohm::kFree,
                                 save_progress_callback);
    break;
  }
  default:
    std::cout << "Invalid mode for point cloud: " << opt.mode << std::endl;
    return -1;
  }

  prog.endProgress();
  prog.pause();
  prog.joinThread();

  return res;
}


int exportHeightmapMesh(const Options &opt, ProgressMonitor &prog, LoadMapProgress &load_progress)
{
  ohm::Heightmap heightmap;
  ohm::PlyMesh ply;
  prog.startThread();
  int res = ohm::load(opt.map_file.c_str(), heightmap, &load_progress);
  prog.endProgress();
  prog.pause();
  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load heightmap. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  ohm::HeightmapMesh mesh;
  mesh.buildMesh(heightmap);
  mesh.extractPlyMesh(ply);

  if (!g_quit)
  {
    if (!ply.save(opt.ply_file.c_str(), true))
    {
      res = -1;
    }
  }

  return res;
}

int exportCovariance(const Options &opt, ProgressMonitor &prog, LoadMapProgress &load_progress)
{
  ohm::OccupancyMap map(1.0f);
  ohm::PlyMesh ply;

  prog.startThread();
  int res = ohm::load(opt.map_file.c_str(), map, &load_progress);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load map. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  // Validate we have covariance and voxel mean.
  if (map.layout().meanLayer() == -1)
  {
    std::cerr << "Missing voxel mean layer" << std::endl;
    res = -1;
  }

  if (map.layout().covarianceLayer() == -1)
  {
    std::cerr << "Missing covariance layer" << std::endl;
    res = -1;
  }

  if (res != 0)
  {
    return res;
  }

  std::vector<glm::dvec3> sphere_verts;
  std::vector<unsigned> sphere_inds;
  makeUnitSphere(sphere_verts, sphere_inds);

  const auto add_ellipsoid = [&sphere_verts, &sphere_inds](ohm::PlyMesh &ply, const glm::dmat4 &transform)  //
  {
    unsigned index_offset = ~0u;

    const ohm::Colour c = ohm::Colour::kColours[ohm::Colour::kSeaGreen];
    for (const auto &v : sphere_verts)
    {
      index_offset = std::min(ply.addVertex(glm::dvec3(transform * glm::dvec4(v, 1.0)), c), index_offset);
    }

    for (size_t i = 0; i < sphere_inds.size(); i += 3)
    {
      ply.addTriangle(sphere_inds[i + 0] + index_offset, sphere_inds[i + 1] + index_offset,
                      sphere_inds[i + 2] + index_offset, c);
    }
  };

  const size_t region_count = map.regionCount();
  glm::i16vec3 last_region = map.begin().key().regionKey();

  ohm::Voxel<const float> occupancy(&map, map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> mean(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::CovarianceVoxel> covariance(&map, map.layout().covarianceLayer());

  if (!occupancy.isLayerValid())
  {
    std::cerr << "Missing " << ohm::default_layer::occupancyLayerName() << " layer" << std::endl;
    res = -1;
  }

  if (!mean.isLayerValid())
  {
    std::cerr << "Missing " << ohm::default_layer::meanLayerName() << " layer" << std::endl;
    res = -1;
  }

  if (!covariance.isLayerValid())
  {
    std::cerr << "Missing " << ohm::default_layer::covarianceLayerName() << " layer" << std::endl;
    res = -1;
  }

  if (res != 0)
  {
    return res;
  }

  prog.beginProgress(ProgressMonitor::Info(region_count));

  for (auto iter = map.begin(); iter != map.end() && !g_quit; ++iter)
  {
    ohm::setVoxelKey(*iter, occupancy, mean, covariance);
    if (last_region != iter->regionKey())
    {
      prog.incrementProgress();
      last_region = iter.key().regionKey();
    }

    if (!ohm::isOccupied(occupancy))
    {
      continue;
    }

    const glm::dvec3 pos = ohm::positionSafe(mean);
    ohm::CovarianceVoxel cov;
    covariance.read(&cov);

    // Add an ellipsoid to the PLY
    glm::dquat rot;
    glm::dvec3 scale;
    ohm::covarianceUnitSphereTransformation(&cov, &rot, &scale);
    // For rendering niceness, we scale up a bit to get better overlap between voxels.
    const double scale_factor = std::sqrt(3.0);
    scale *= scale_factor;

    const glm::dmat4 transform = glm::translate(pos) * glm::mat4_cast(rot) * glm::scale(scale);
    add_ellipsoid(ply, transform);
  }

#if OHM_COV_DEBUG
  ohm::covDebugStats();
#endif  // OHM_COV_DEBUG

  if (!g_quit)
  {
    if (!ply.save(opt.ply_file.c_str(), true))
    {
      res = -1;
    }
  }

  return res;
}

int main(int argc, char *argv[])
{
  Options opt;
  std::cout.imbue(std::locale(""));

  int res = parseOptions(&opt, argc, argv);

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

  prog.setDisplayFunction([](const ProgressMonitor::Progress &prog) {
    // if (!opt.quiet)
    {
      std::ostringstream out;
      out.imbue(std::locale(""));
      out << '\r';

      if (!prog.info.info.empty())
      {
        out << prog.info.info << " : ";
      }

      const auto fill_width = std::numeric_limits<decltype(prog.progress)>::digits10;
      out << std::setfill(' ') << std::setw(fill_width) << prog.progress;
      if (prog.info.total)
      {
        out << " / " << std::setfill(' ') << std::setw(fill_width) << prog.info.total;
      }
      out << "    ";
      std::cout << out.str() << std::flush;
    }
  });

  switch (opt.mode)
  {
  case kExportOccupancy:
  case kExportOccupancyCentre:
  case kExportObserved:
  case kExportDecay:
  case kExportClearance:
  case kExportHeightmap:
    res = exportPointCloud(opt, prog, load_progress);
    break;
  case kExportHeightmapMesh:
    res = exportHeightmapMesh(opt, prog, load_progress);
    break;
  case kExportCovariance:
    res = exportCovariance(opt, prog, load_progress);
    break;
  }

  prog.endProgress();
  prog.pause();
  prog.joinThread();

  return res;
}
