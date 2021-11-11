// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// This is the program file for populating an Octomap occupancy map using SLAM data.
// The main entry point is near the bottom of the file with additional option parsing and support functions above that.
//
#include "OctoPopConfig.h"

#include <ohmapp/OhmPopulationHarness.h>
#include <ohmapp/ohmpopmain.inl>

#include <ohmutil/Colour.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyPointStream.h>

#include <octomap/octomap.h>

#include <glm/vec2.hpp>

#include <chrono>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

using Clock = std::chrono::high_resolution_clock;

class OctomapPop : public ohm::OhmPopulationHarness
{
public:
  using Super = ohm::OhmPopulationHarness;

  struct MapOptions : public Super::MapOptions
  {
    float prob_hit = 0.9f;
    float prob_miss = 0.49f;
    float prob_thresh = 0.5f;
    glm::vec2 prob_range = glm::vec2(0, 0);
    bool non_lazy_eval = false;
    bool collapse = false;

    MapOptions();

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    inline MapOptions &map() { return static_cast<MapOptions &>(*map_); }
    inline const MapOptions &map() const { return static_cast<const MapOptions &>(*map_); }

    Options();
  };

  /// Default constructor.
  OctomapPop();

  std::string description() const override;

  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  Options &options() { return static_cast<Options &>(Super::options()); }

protected:
  OctomapPop(std::unique_ptr<Options> &&options);

  int validateOptions(const cxxopts::ParseResult &parsed) override;
  int prepareForRun() override;
  void processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                    const std::vector<double> &timestamps, const std::vector<float> &intensities,
                    const std::vector<glm::vec4> &colours) override;
  void finaliseMap() override;
  int saveMap(const std::string &path_without_extension) override;
  int saveCloud(const std::string &path_ply) override;
  void tearDown() override;

  std::unique_ptr<octomap::OcTree> map_;
};


OctomapPop::MapOptions::MapOptions() = default;


void OctomapPop::MapOptions::configure(cxxopts::OptionAdder &adder)
{
  Super::MapOptions::configure(adder);
  // clang-format off
  adder
    ("clamp", "Set probability clamping to the given min/max. Given as a value, not probability.", optVal(prob_range))
    ("collapse", "Collapse the map once fully populated? No effect when --non-lazy is also used.", optVal(collapse))
    ("hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(prob_hit))
    ("non-lazy", "Use non-lazy evaluation, collapsing the map on every batch.", optVal(non_lazy_eval))
    ("miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(prob_miss))
    ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(prob_thresh)->implicit_value(optStr(prob_thresh)))
    ;
  // clang-format on
}


void OctomapPop::MapOptions::print(std::ostream &out)
{
  Super::MapOptions::print(out);
  out << "Hit probability: " << prob_hit << " (" << octomap::logodds(prob_hit) << ")\n";
  out << "Miss probability: " << prob_miss << " (" << octomap::logodds(prob_miss) << ")\n";
  out << "Probability threshold: " << prob_thresh << '\n';
}


OctomapPop::Options::Options()
{
  map_ = std::make_unique<MapOptions>();
}

OctomapPop::OctomapPop()
  : Super(std::make_unique<Options>())
{}

OctomapPop::OctomapPop(std::unique_ptr<Options> &&options)
  : Super(std::move(options))
{}


#if SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PDAL supported point cloud"
#else  // SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PLY point cloud"
#endif  // SLAMIO_HAVE_PDAL

std::string OctomapPop::description() const
{
  return "Generate an octomap octree from a ray cloud or a point cloud with accompanying "
         "trajectory file. The trajectory marks the scanner trajectory with timestamps "
         "loosely corresponding to cloud point timestamps. Trajectory points are "
         "interpolated for each cloud point based on corresponding times in the "
         "trajectory. A ray cloud uses the normals channel to provide a vector from "
         "point sample back to sensor location (see "
         "https://github.com/csiro-robotics/raycloudtools).\n"
         "\n"
         "The sample file is a " CLOUD_TYPE " file, while the trajectory is either a text "
         "trajectory containing [time x y z <additional>] items per line or is itself a "
         "point cloud file.";
}


int OctomapPop::validateOptions(const cxxopts::ParseResult &parsed)
{
  int return_code = Super::validateOptions(parsed);
  if (return_code)
  {
    return return_code;
  }
  return 0;
}


int OctomapPop::prepareForRun()
{
  map_ = std::make_unique<octomap::OcTree>(options().map().resolution);
  map_->setProbHit(options().map().prob_hit);
  map_->setOccupancyThres(options().map().prob_thresh);
  map_->setProbMiss(options().map().prob_miss);
  if (options().map().prob_range[0] > 0)
  {
    map_->setClampingThresMin(options().map().prob_range[0]);
  }
  if (options().map().prob_range[1] > 0)
  {
    map_->setClampingThresMax(options().map().prob_range[1]);
  }
  return 0;
}


void OctomapPop::processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                              const std::vector<double> &timestamps, const std::vector<float> &intensities,
                              const std::vector<glm::vec4> &colours)
{
  // Unused
  (void)batch_origin;
  (void)timestamps;
  (void)intensities;
  (void)colours;
  // Note: we do not use octomap's batch ray integration for two reasons:
  // 1. This allows us to use individual sensor positions for each sample
  // 2. It tries to use OpenMP, but in a way which is empirically slower than using a single thread.
  const bool lazy_eval = !options().map().non_lazy_eval;
  for (size_t i = 0; i < sensor_and_samples.size(); i += 1)
  {
    const auto sensor = sensor_and_samples[i + 0];
    const auto sample = sensor_and_samples[i + 1];
    map_->insertRay(octomap::point3d{ float(sensor.x), float(sensor.y), float(sensor.z) },
                    octomap::point3d{ float(sample.x), float(sample.y), float(sample.z) }, -1.0, lazy_eval);
  }
}


void OctomapPop::finaliseMap()
{
  if (!options().map().non_lazy_eval && options().map().collapse)
  {
    info("Collapsing map\n");
    const auto collapse_start_time = Clock::now();
    map_->updateInnerOccupancy();
    const auto collapse_end_time = Clock::now();
    std::ostringstream ostr;
    ostr << "Collapsed time: " << (collapse_end_time - collapse_start_time) << std::endl;
    info(ostr.str());
  }
}


int OctomapPop::saveMap(const std::string &path_without_extension)
{
  const std::string output_file = path_without_extension + ".bt";
  std::ostringstream ostr;
  ostr << "Saving map to " << output_file.c_str() << std::endl;
  info(ostr.str());

  bool ok = map_->writeBinary(output_file);
  if (!ok)
  {
    error("Failed to save map\n");
    return -1;
  }
  return 0;
}


int OctomapPop::saveCloud(const std::string &path_ply)
{
  // Save a cloud representation. Need to walk the tree leaves.
  std::ostringstream out;
  out.imbue(std::locale(""));
  out << "Converting to point cloud " << path_ply << std::endl;
  info(out.str());
  ohm::PlyPointStream ply({ ohm::PlyPointStream::Property{ "x", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "y", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "z", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "red", ohm::PlyPointStream::Type::kUInt8 },
                            ohm::PlyPointStream::Property{ "green", ohm::PlyPointStream::Type::kUInt8 },
                            ohm::PlyPointStream::Property{ "blue", ohm::PlyPointStream::Type::kUInt8 } });
  std::ofstream fout(path_ply, std::ios::binary);
  ply.open(fout);

  const bool occupancy_colour = options().output().cloud_colour.r > 0 || options().output().cloud_colour.g > 0 ||
                                options().output().cloud_colour.b > 0;
  const ohm::Colour c = ohm::Colour::fromRgbf(options().output().cloud_colour.r, options().output().cloud_colour.g,
                                              options().output().cloud_colour.b);

  const auto colour_by_occupancy = [](float occupancy) {
    ohm::Colour colour = ohm::Colour::kColours[ohm::Colour::kLightSteelBlue];
    // Occupancy will be at least the occupancy threshold (normaly 50%) so this will only darken to that level.
    colour.adjust(occupancy);
    return colour;
  };

  const auto map_end_iter = map_->end_leafs();
  for (auto iter = map_->begin_leafs(); iter != map_end_iter && quitLevel() < 2; ++iter)
  {
    const float occupancy = float(iter->getOccupancy());
    if (occupancy >= map_->getOccupancyThres())
    {
      const auto coord = iter.getCoordinate();
      const auto point = glm::dvec3(coord.x(), coord.y(), coord.z());

      const ohm::Colour point_colour = (occupancy_colour) ? c : colour_by_occupancy(occupancy);

      ply.setPointPosition(point);
      ply.setProperty("red", point_colour.r());
      ply.setProperty("green", point_colour.g());
      ply.setProperty("blue", point_colour.b());
      ply.writePoint();
    }
  }

  ply.close();
  out.str(std::string());
  out << ply.pointCount() << " point(s) saved" << std::endl;
  info(out.str());
  return 0;
}


void OctomapPop::tearDown()
{
  map_.release();
}

int main(int argc, char *argv[])
{
  return ohmpopMain<OctomapPop>(argc, argv);
}
