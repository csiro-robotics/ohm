//
// author Kazys Stepanas
//
#include <glm/glm.hpp>

#include <ohm/Heightmap.h>
#include <ohm/HeightmapVoxel.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/Voxel.h>

#include <ohmheightmaputil/HeightmapImage.h>

#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyMesh.h>
#include <ohmutil/ProgressMonitor.h>
#include <ohmutil/SafeIO.h>
#include <ohmutil/ScopedTimeDisplay.h>

#include <ohmutil/Options.h>

#include <png.h>

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
    std::string image_file;
  };


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

  bool savePng(const char *filename, const uint8_t *raw, const ohm::HeightmapImage::BitmapInfo &info)
  {
    png_image image;

    // Initialize the 'png_image' structure.
    memset(&image, 0, sizeof(image));
    image.version = PNG_IMAGE_VERSION;
    image.width = info.image_width;
    image.height = info.image_height;
    image.flags = 0;
    image.colormap_entries = 0;

    int row_stride = 0;
    std::vector<uint16_t> u16_pixels;
    const uint8_t *image_data = raw;
    if (info.type == ohm::HeightmapImage::kImageNormals)
    {
      // Need to convert float colour to u16
      u16_pixels.reserve(info.image_width * info.image_height * 3);

      float red, green, blue;
      uint16_t red16, green16, blue16;

      const auto convert_colour = [] (float c) -> uint16_t
      {
        return uint16_t(0.5f * (1.0f + c) * float(0xffffu));
      };

      for (size_t i = 0; i < info.image_width * info.image_height * info.bpp; i += 3 * sizeof(float))
      {
        red = *reinterpret_cast<const float *>(raw + i);
        green = *reinterpret_cast<const float *>(raw + i + sizeof(float));
        blue = *reinterpret_cast<const float *>(raw + i + 2 * sizeof(float));

        red16 = convert_colour(red);
        green16 = convert_colour(green);
        blue16 = convert_colour(blue);

        // No data: black
        if (red * red + green * green + blue * blue < 0.5f)
        {
          red16 = green16 = blue16 = 0;
        }

        u16_pixels.push_back(red16);
        u16_pixels.push_back(green16);
        u16_pixels.push_back(blue16);
      }

      image.format = PNG_FORMAT_RGB | PNG_IMAGE_FLAG_16BIT_sRGB;
      // 16 bits per pixel.
      // image.flags |= PNG_IMAGE_FLAG_16BIT_sRGB;
      row_stride = -int(info.image_width * 3);
      image_data = reinterpret_cast<const uint8_t *>(u16_pixels.data());
    }
    else if (info.type == ohm::HeightmapImage::kImageNormals888)
    {
      image.format = PNG_FORMAT_RGB;
      row_stride = -int(info.image_width * 3);
    }
    else if (info.bpp == sizeof(float))
    {
      u16_pixels.resize(info.image_width * info.image_height);
      float depth;
      for (size_t i = 0; i < info.image_width * info.image_height * info.bpp; i += info.bpp)
      {
        depth = *reinterpret_cast<const float *>(raw + i);
        u16_pixels[i] = uint16_t(1.0f - depth * float(0xffffu));
      }

      image.format = PNG_FORMAT_LINEAR_Y;
      row_stride = -int(info.image_width);
      image_data = reinterpret_cast<const uint8_t *>(u16_pixels.data());
    }
    else
    {
      return false;
    }

    // Negative row stride to flip the image.
    if (png_image_write_to_file(&image, filename, false,  // convert_to_8bit,
                                image_data,
                                row_stride,  // row_stride
                                nullptr                                 // colormap
                                ))
    {
      return true;
    }

    return false;
  }
}  // namespace


int parseOptions(Options &opt, int argc, char *argv[])
{
  cxxopts::Options optParse(argv[0], "\nCreate a heightmap from an occupancy map.\n");
  optParse.positional_help("<map.ohm> <heightmap.ohm>");

  try
  {
    optParse.add_options()("help", "Show help.")("i", "The input heightmap file (ohm).", cxxopts::value(opt.map_file))(
      "o", "The output heightmap image file (png).", cxxopts::value(opt.image_file));

    optParse.parse_positional({ "i", "o" });

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

    if (opt.image_file.empty())
    {
      std::cerr << "Missing output name" << std::endl;
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
  ProgressMonitor prog(10);
  LoadMapProgress load_progress(prog);
  ohm::Heightmap heightmap;
  ohm::MapVersion version;

  prog.setDisplayFunction([&opt](const ProgressMonitor::Progress &prog) {
    std::ostringstream str;
    str << '\r';
    str << prog.progress;
    if (prog.info.total)
    {
      str << " / " << prog.info.total;
    }
    str << "      ";
    std::cout << str.str() << std::flush;
  });

  prog.startThread();
  res = ohm::load(opt.map_file.c_str(), heightmap, &load_progress, &version);
  prog.endProgress();

  std::cout << std::endl;

  if (res != 0)
  {
    std::cerr << "Failed to load heightmap. Error(" << res << "): " << ohm::errorCodeString(res) << std::endl;
    return res;
  }

  ohm::HeightmapImage hmImage(heightmap);
  ohm::HeightmapImage::BitmapInfo info;
  hmImage.generateBitmap();
  const uint8_t *image = hmImage.bitmap(&info);
  if (!image)
  {
    std::cerr << "Failed to generate heightmap image" << std::endl;
    return 1;
  }

  std::cout << "Saving " << opt.image_file << std::endl;
  if (!savePng(opt.image_file.c_str(), image, info))
  {
    std::cerr << "Failed to save heightmap image" << std::endl;
    return 1;
  }

  return res;
}
