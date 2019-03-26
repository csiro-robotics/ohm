// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmTestConfig.h"

#include "RayValidation.h"

#include <ohm/RayPatternConical.h>

#include <3esservermacros.h>

#include <cstdio>
#include <vector>

#include <glm/ext.hpp>

#include <gtest/gtest.h>

#ifdef OHM_WITH_PNG
#include <png.h>
#endif  // OHM_WITH_PNG

using namespace ohm;

namespace general
{
#ifdef OHM_WITH_PNG
  bool savePng(const char *filename, const std::vector<uint8_t> &raw, unsigned w, unsigned h)
  {
    png_image image;

    // Initialize the 'png_image' structure.
    memset(&image, 0, sizeof(image));
    image.version = PNG_IMAGE_VERSION;
    image.width = int(w);
    image.height = int(h);
    image.flags = 0;
    image.colormap_entries = 0;

    int row_stride = w;
    image.format = PNG_FORMAT_GRAY;
    row_stride = -int(w);

    // Negative row stride to flip the image.
    if (png_image_write_to_file(&image, filename, false,  // convert_to_8bit,
                                raw.data(),
                                row_stride,  // row_stride
                                nullptr      // colormap
                                ))
    {
      return true;
    }

    return false;
  }
#endif  // OHM_WITH_PNG

  // Debug function: log the image for generating comparison data. This essentially makes a regression test, but with
  // visual confirmation of the image from savePng().
  void logImage(const std::vector<uint8_t> &image, unsigned width, unsigned height)
  {
    std::cout << "const size_t image_size = " << width * height << ";\n";
    std::cout << "const uint8_t image[" << width * height << "] = //\n";
    std::cout << "{ //";
    const auto restore_width = std::cout.width();
    const auto restore_fill = std::cout.fill();
    const size_t bytes_per_line = 20;
    for (size_t i = 0; i < image.size(); ++i)
    {
      if (i != 0)
      {
        std::cout << ',';
      }
      if (i % bytes_per_line == 0)
      {
        std::cout << "\n  ";
      }
      std::cout << "0x" << std::setfill('0') << std::setw(2) << std::hex << unsigned(image[i]);
    }
    std::cout.width(restore_width);
    std::cout.fill(restore_fill);
    std::cout << "\n};\n";
    std::cout << std::endl;
  }

  void validateImage(const std::vector<uint8_t> &image, unsigned width, unsigned height, const uint8_t *reference,
                     size_t reference_element_count)
  {
    ASSERT_EQ(width * height, reference_element_count);
    ASSERT_EQ(image.size(), reference_element_count);

    for (size_t i = 0; i < reference_element_count; ++i)
    {
      ASSERT_EQ(image[i], reference[i]);
    }
  }

  TEST(General, RayPattern)
  {
    const double angular_resolution = glm::radians(5.0);
    RayPatternConical pattern(glm::dvec3(0.7, 0.7, 0), glm::radians(65.0), 5.0, angular_resolution);

    // To test the ray pattern, we will convert the rays to a spherical image and validate we have a circle where we
    // expect.
    std::vector<uint8_t> spherical_image;
    const unsigned image_height = unsigned(std::ceil(M_PI / angular_resolution));
    const unsigned image_width = unsigned(std::ceil(2.0 * M_PI / angular_resolution));
    spherical_image.resize(image_height * image_width);

    // Clear the image.
    std::fill(spherical_image.begin(), spherical_image.end(), uint8_t(0));

    // Access the ray pattern and quantise in polar space.
    const size_t ray_count = pattern.pointCount();
    const glm::dvec3 *ray_end = pattern.points();
    for (size_t i = 0; i < ray_count; ++i, ++ray_end)
    {
      glm::dvec2 ray_xy(ray_end->x, ray_end->y);
      const double scale_2d = glm::length(ray_xy);
      ray_xy *= (scale_2d) ? 1.0 / scale_2d : 0.0;
      double azimuth = std::atan2(ray_xy.y, ray_xy.x);
      double elevation = std::atan2(ray_end->z, scale_2d);

      // Ensure azimuth is in the range [0, 2pi]
      if (azimuth < 0)
      {
        azimuth += 2.0 * M_PI;
      }

      // Quantise.
      int azimuth_bucket = int(azimuth / angular_resolution);
      // Ensure azimuth is in the range [0, pi]
      int elevation_bucket = int((elevation + 0.5 * M_PI) / angular_resolution);

      ASSERT_GE(azimuth_bucket, 0);
      ASSERT_LT(azimuth_bucket, int(image_width));
      ASSERT_GE(elevation_bucket, 0);
      ASSERT_LT(elevation_bucket, int(image_height));

      const unsigned pixel_index = unsigned(azimuth_bucket) + unsigned(elevation_bucket) * image_width;
      ASSERT_LT(pixel_index, unsigned(spherical_image.size()));
      spherical_image[pixel_index] = uint8_t(255u);
    }

#ifdef OHM_WITH_PNG
    savePng("ray-conical.png", spherical_image, image_width, image_height);
#endif  // OHM_WITH_PNG

    // logImage(spherical_image, image_width, image_height);

    validateImage(spherical_image, image_width, image_height, ohm::ray::conical_ray_image,
                  ohm::ray::conical_ray_image_size);
  }
}  // namespace general
