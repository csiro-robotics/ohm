// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMAPLAYER_H_
#define OHMMAPLAYER_H_

#include "ohmconfig.h"

#include "ohmvoxellayout.h"

#include <glm/fwd.hpp>

namespace ohm
{
  struct MapLayerDetail;
  struct MapChunk;

  /// Defines a layer in a @c MapLayout.
  ///
  /// See @c MapLayout for details.
  class MapLayer
  {
  public:
    /// Configuration flags.
    enum Flag
    {
      /// Layer data is not serialised to disk.
      SkipSerialise = (1 << 0)
    };

    /// Construct a new layer.
    /// @param name The layer name.
    /// @param layerIndex The layer index in @p MapLayout.
    /// @param subsampling Voxel downsampling factor.
    MapLayer(const char *name, unsigned layerIndex = 0, unsigned subsampling = 0);

    /// Destructor.
    ~MapLayer();

    /// Clears the layer details.
    void clear();

    /// Access the name.
    /// @return The layer name.
    const char *name() const;

    /// Access the layer index.
    /// @return The layer index in it's @c MapLayout.
    unsigned layerIndex() const;

    /// Access the layer subsampling factor.
    /// @return The layer subsampling factor.
    unsigned subsampling() const;

    /// Access the layer flags.
    /// @return The layer @c Flag values.
    unsigned flags() const;

    /// Set the layer flags.
    /// @param flags New flags to set.
    void setFlags(unsigned flags);

    /// Copy the @c VoxelLayout from @p other.
    /// @param other Layer to copy the voxel structure of.
    void copyVoxelLayout(const MapLayer &other);

    /// Read only access to the voxel layout of this layer.
    /// @return Voxel layout details.
    VoxelLayoutConst voxelLayout() const;

    /// Writable access to the voxel layout of this layer.
    /// @return Voxel layout configuration.
    VoxelLayout voxelLayout();

    /// Gives the per chunk voxel dimensions of this layer given @p regionDim defines the maximum voxel dimensions.
    ///
    /// For each magnitude step of @c subsampling(), each element of @p regionDim is halved to a minium of 1.
    ///
    /// @param regionDim The dimensions of each chunk the owning @c OccupancyMap.
    /// @return The voxel dimensions for this layer based on @p regionDim.
    glm::u8vec3 dimensions(const glm::u8vec3 &regionDim) const;

    /// Retrieve the volume of voxels in each chunk for this layer.
    ///
    /// This is the same as the volume of @c dimensions().
    ///
    /// @param regionDim The dimensions of each chunk the owning @c OccupancyMap.
    /// @return The voxel volume for this layer based on @p regionDim.
    size_t volume(const glm::u8vec3 &regionDim) const;

    /// Query the size of each voxel in this layer in bytes.
    ///
    /// Same as @c voxelLayout()->voxelByteSize().
    ///
    /// @return The size of each voxel in this layer.
    size_t voxelByteSize() const;

    /// Query the byte size this layer in each each @c MapChunk.
    /// @return The size of this layer in each @p MapChunk.
    size_t layerByteSize(const glm::u8vec3 &regionDim) const;

    /// Allocate memory for this layer given @p regionDim.
    /// @param regionDim The dimensions of each chunk the owning @c OccupancyMap.
    /// @return An uninitialised memory block of size @c layerByteSize(regionDim).
    uint8_t *allocate(const glm::u8vec3 &regionDim) const;

    /// Release memory previously allocated by @c allocate().
    /// @param voxels Memory previously allocated by @c allocate().
    void release(const uint8_t *voxels) const;

    /// Clear the given layer memory to the default value.
    /// The default values are set by the @c VoxelLayout details.
    /// @param mem The memory to clear, previously allocated from @c allocate().
    /// @param regionDim The dimensions of each region with no subsampling. Subsampling is resolved as needed.
    void clear(uint8_t *mem, const glm::u8vec3 &regionDim) const;

    /// Get a pointer the the voxel data for this layer in @p chunk.
    /// @param chunk The map chunk to get layer data for.
    /// @return The data for this layer in @p chunk.
    const uint8_t *voxels(const MapChunk &chunk) const;

    /// @overload
    uint8_t *voxels(MapChunk &chunk) const;

    /// Get a pointer the the voxel data for this layer in @p chunk cast as type @c T.
    ///
    /// Validates that the size of @c T matches the @c voxelByteSize().
    ///
    /// @tparam T The type to cast to.
    /// @param chunk The map chunk to get layer data for.
    /// @return The data for this layer in @p chunk or null on a size mismatch.
    template <typename T> const T *voxelsAs(const MapChunk &chunk) const;

    /// @overload
    template <typename T> T *voxelsAs(MapChunk &chunk) const;

  private:
    MapLayerDetail *_detail;
  };


  template <typename T>
  inline const T *MapLayer::voxelsAs(const MapChunk &chunk) const
  {
    if (voxelByteSize() == sizeof(T))
    {
      return reinterpret_cast<const T *>(voxels(chunk));
    }
    return nullptr;
  }


  template <typename T>
  inline T *MapLayer::voxelsAs(MapChunk &chunk) const
  {
    if (voxelByteSize() == sizeof(T))
    {
      return reinterpret_cast<T *>(voxels(chunk));
    }
    return nullptr;
  }
}

#endif // OHMMAPLAYER_H_
