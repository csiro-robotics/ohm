// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef HEIGHTMAPIMAGE_H
#define HEIGHTMAPIMAGE_H

#include "OhmHeightmapUtilConfig.h"

#include <glm/fwd.hpp>

#include <memory>

namespace ohm
{
  class Heightmap;

  struct HeightmapImageDetail;

  class ohmheightmaputil_API HeightmapImage
  {
  public:
    struct BitmapInfo
    {
      unsigned image_width;
      unsigned image_height;
      unsigned bpp;
      size_t byteCount;
    };

    HeightmapImage(const Heightmap &heightmap);
    ~HeightmapImage();

    /// Query the number of bytes required to extract the heightmap into a bitmap using @c extractBitmap().
    /// @return The number of bytes required for the bitmap.
    bool bitmapInfo(BitmapInfo *info) const;

    /// Extract the heightmap into a bitmap buffer. The bitmap format is a grey scale, 1 byte per pixel bitmap image.
    ///
    /// Pixels are coloured by the slope of a pixel.
    ///
    /// Voxel Type  | Colour    | Pixel Value
    /// ----------- |
    /// Uncertain   | Black     | 0
    ///
    bool extractBitmap(uint8_t *buffer, size_t buffer_size, BitmapInfo *info);

  private:
    void triangulate(glm::dvec3 *min_ext, glm::dvec3 *max_ext);
    bool renderHeightMesh(const glm::dvec3 &min_ext_spatial, const glm::dvec3 &max_ext_spatial, double voxel_resolution);

    std::unique_ptr<HeightmapImageDetail> imp_;
  };
}

#endif // HEIGHTMAPIMAGE_H
