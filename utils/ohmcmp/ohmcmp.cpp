// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

// ohmcmp is an experimental utility for comparing two ohm maps.

#include <glm/glm.hpp>

#include <ohm/CompareMaps.h>
#include <ohm/DefaultLayer.h>
#include <ohm/MapInfo.h>
#include <ohm/MapLayer.h>
#include <ohm/MapLayout.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/VoxelData.h>
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
#include <unordered_map>
#include <vector>

namespace
{
struct Options
{
  std::string input_map_file;
  std::string ref_map_file;
  std::vector<std::string> layers;
  unsigned verbosity = 1;
  bool compare_layout = false;
  bool compare_voxels = false;
  bool stop_on_error = false;
  bool tolerances = false;
};
}  // namespace


int parseOptions(Options *opt, int argc, char *argv[])  // NOLINT(modernize-avoid-c-arrays)
{
  cxxopts::Options opt_parse(argv[0], "\nAn experimental tool for comparing two ohm maps.\n");
  opt_parse.positional_help("<map.ohm>");

  try
  {
    // clang-format off
    opt_parse.add_options()
      ("help", "Show help.")
      ("input", "The input map file (ohm) to validate.", cxxopts::value(opt->input_map_file))
      ("ref", "The reference map file (ohm) to validate against.", cxxopts::value(opt->ref_map_file))
      ("layers", "List of layers to limit comparison to. Affects layout and voxel comparison.", cxxopts::value(opt->layers))
      ("layout", "Compare map layouts and report differences?", optVal(opt->compare_layout))
      ("stop-on-error", "Stop on the first error?", optVal(opt->stop_on_error))
      ("tolerances", "Allow some error tolerance?.", optVal(opt->tolerances))
      ("verbosity", "Verbosity level [0, 2].", optVal(opt->verbosity))
      ("voxels", "Compare voxel content? Limited to the list of layers specified.", optVal(opt->compare_voxels))
      ;
    // clang-format on

    opt_parse.parse_positional({ "input", "ref" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help({ "", "Group" }) << std::endl;
      return 1;
    }

    if (opt->input_map_file.empty())
    {
      std::cerr << "Missing input map" << std::endl;
      return -1;
    }

    if (opt->ref_map_file.empty())
    {
      std::cerr << "Missing reference map" << std::endl;
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

using ToleranceMap = std::unordered_map<std::string, std::shared_ptr<ohm::MapLayer>>;

void buildDefaultLayerTolerances(ToleranceMap &map)
{
  // TODO(KS): validate the default tolerances.
  // Build a map of tolerances for the default layers.

  // Occupancy layer
  std::shared_ptr<ohm::MapLayer> layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::occupancyLayerName());
  ohm::compare::configureTolerance(*layer, "occupancy", 1e2f);
  map.emplace(std::make_pair(std::string(layer->name()), layer));

  // Cannot support a difference in the quantised mean value.
  // // Mean layer
  // layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::meanLayerName());
  // ohm::compare::configureTolerance(*layer, "count", uint32_t(1));
  // map.emplace(std::make_pair(std::string(layer->name()), layer));

  // Mean layer
  layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::covarianceLayerName());
  // Cannot support a difference in the quantised mean value.
  const float p_err = 1e-2f;
  ohm::compare::configureTolerance(*layer, "P00", p_err);
  ohm::compare::configureTolerance(*layer, "P01", p_err);
  ohm::compare::configureTolerance(*layer, "P11", p_err);
  ohm::compare::configureTolerance(*layer, "P02", p_err);
  ohm::compare::configureTolerance(*layer, "P12", p_err);
  ohm::compare::configureTolerance(*layer, "P22", p_err);
  map.emplace(std::make_pair(std::string(layer->name()), layer));

  layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::clearanceLayerName());
  ohm::compare::configureTolerance(*layer, "clearance", 1e-3f);
  map.emplace(std::make_pair(std::string(layer->name()), layer));

  layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::intensityLayerName());
  ohm::compare::configureTolerance(*layer, "mean", 1e-3f);
  ohm::compare::configureTolerance(*layer, "cov", 1e-3f);
  map.emplace(std::make_pair(std::string(layer->name()), layer));

  layer = std::make_shared<ohm::MapLayer>(ohm::default_layer::intensityLayerName());
  ohm::compare::configureTolerance(*layer, "mean", 1e-3f);
  ohm::compare::configureTolerance(*layer, "cov", 1e-3f);
  map.emplace(std::make_pair(std::string(layer->name()), layer));
}

// Extract the list of layers in map
std::vector<std::string> buildLayerList(const ohm::OccupancyMap &map)
{
  std::vector<std::string> layers;

  const auto &layout = map.layout();

  for (size_t i = 0; i < layout.layerCount(); ++i)
  {
    layers.emplace_back(layout.layer(i).name());
  }

  return layers;
}

// Remove items from layers which are not also in filter.
void filterLayers(std::vector<std::string> *layers, const std::vector<std::string> &filter)
{
  for (auto iter = layers->begin(); iter != layers->end(); /*no op*/)
  {
    if (std::find(filter.begin(), filter.end(), *iter) != filter.end())
    {
      // Keep this item.
      ++iter;
    }
    else
    {
      // Remove this item.
      iter = layers->erase(iter);
    }
  }
}

int main(int argc, char *argv[])
{
  Options opt;

  std::cout.imbue(std::locale(""));

  int res = 0;
  res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  ohm::OccupancyMap input_map(1.0f);
  ohm::OccupancyMap ref_map(1.0f);

  ohm::MapVersion input_version{};
  ohm::MapVersion ref_version{};

  size_t input_region_count = 0;
  size_t ref_region_count = 0;

  // Decide whether we need to load the map content, or just the headers.
  if (opt.compare_voxels)
  {
    // Full load.
    std::cout << "Loading input map " << opt.input_map_file << std::endl;
    res = ohm::load(opt.input_map_file, input_map, nullptr, &input_version);
    if (res != 0)
    {
      std::cout << "Loading failed" << std::endl;
      return res;
    }
    std::cout << "Loading reference map " << opt.ref_map_file << std::endl;
    res = ohm::load(opt.ref_map_file, ref_map, nullptr, &ref_version);
    if (res != 0)
    {
      std::cout << "Loading failed" << std::endl;
      return res;
    }

    input_region_count = input_map.regionCount();
    ref_region_count = ref_map.regionCount();
  }
  else
  {
    // Load header only
    std::cout << "Loading input map header " << opt.input_map_file << std::endl;
    res = ohm::loadHeader(opt.input_map_file, input_map, &input_version, &input_region_count);
    if (res != 0)
    {
      std::cout << "Loading failed" << std::endl;
      return res;
    }
    std::cout << "Loading reference map header " << opt.ref_map_file << std::endl;
    res = ohm::loadHeader(opt.ref_map_file, ref_map, &ref_version, &ref_region_count);
    if (res != 0)
    {
      std::cout << "Loading failed" << std::endl;
      return res;
    }
  }


  std::vector<std::string> input_layer_list = buildLayerList(input_map);
  std::vector<std::string> ref_layer_list = buildLayerList(ref_map);

  if (!opt.layers.empty())
  {
    filterLayers(&input_layer_list, opt.layers);
    filterLayers(&ref_layer_list, opt.layers);
  }

  const auto log = +[](ohm::compare::Severity severity, const std::string &msg)  //
  {
    std::ostream *out = &std::cout;
    std::string prefix;
    switch (severity)
    {
    case ohm::compare::Severity::kWarning:
      out = &std::cerr;
      prefix = "warning : ";
      break;
    case ohm::compare::Severity::kError:
      out = &std::cerr;
      prefix = "error : ";
      break;
    default:
      break;
    }

    *out << prefix << msg << std::endl;
  };

  std::array<ohm::compare::Log, 3> logs;
  for (size_t i = 0; i < logs.size(); ++i)
  {
    logs[i] = (opt.verbosity >= i) ? log : ohm::compare::emptyLog;
  }

  if (input_layer_list != ref_layer_list)
  {
    std::ostringstream str;
    str << "Layer mismatch\n";
    str << "  input map:\n";
    for (const auto &layer : input_layer_list)
    {
      str << "    " << layer << '\n';
    }
    str << "  reference map:\n";
    for (const auto &layer : ref_layer_list)
    {
      str << "    " << layer << '\n';
    }
    log(ohm::compare::Severity::kWarning, str.str());
  }

  unsigned compare_flags = 0;
  compare_flags |= ohm::compare::kContinue * !opt.stop_on_error;

  bool ok = true;

  if (opt.compare_layout)
  {
    std::cout << "Comparing layers" << std::endl;
    bool layers_ok = true;
    for (const auto &layer_name : input_layer_list)
    {
      const bool layer_ok = ohm::compare::compareLayoutLayer(input_map, ref_map, layer_name, compare_flags, logs[1]);
      layers_ok = layer_ok && layers_ok;
      std::cout << layer_name << ' ' << (layer_ok ? "ok" : "failed") << std::endl;
    }
    std::cout << "Layers " << (layers_ok ? "ok" : "failed") << std::endl;
    ok = ok && layers_ok;
  }

  if (opt.compare_voxels)
  {
    std::cout << "Comparing voxels" << std::endl;
    bool voxels_ok = true;

    ToleranceMap tolerances;
    if (opt.tolerances)
    {
      buildDefaultLayerTolerances(tolerances);
    }
    for (const auto &layer_name : input_layer_list)
    {
      std::shared_ptr<ohm::MapLayer> tolerance = tolerances[layer_name];

      auto voxel_result =
        ohm::compare::compareVoxels(input_map, ref_map, layer_name, tolerance.get(), compare_flags, logs[2]);
      voxels_ok = voxel_result.layout_match && voxel_result.voxels_failed == 0 && voxels_ok;
      std::cout << layer_name << " layout " << (voxel_result.layout_match ? "ok" : "failed") << " passed "
                << voxel_result.voxels_passed << " failed " << voxel_result.voxels_failed << std::endl;
    }
    std::cout << "Voxels " << (voxels_ok ? "ok" : "failed") << std::endl;
    ok = ok && voxels_ok;
  }

  if (!ok)
  {
    res = 1;
  }

  return res;
}
