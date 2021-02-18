// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPLAYER_H
#define OHM_MAPLAYER_H

#include "OhmConfig.h"

#include "MapChunk.h"
#include "MapLayoutMatch.h"
#include "VoxelLayout.h"

#include <glm/vec3.hpp>

#include <memory>
#include <string>

namespace ohm
{
struct MapChunk;
struct VoxelLayoutDetail;

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
    kSkipSerialise = (1u << 0u)
  };

  /// Construct a new layer.
  /// @param name The layer name.
  /// @param layer_index The layer index in @p MapLayout.
  /// @param subsampling Voxel downsampling factor.
  explicit MapLayer(const char *name, unsigned layer_index = 0, unsigned subsampling = 0);

  /// Destructor.
  ~MapLayer();

  /// Clears the layer details.
  void clear();

  /// Access the name.
  /// @return The layer name.
  inline const char *name() const { return name_.c_str(); }

  /// Access the layer index.
  /// @return The layer index in it's @c MapLayout.
  inline unsigned layerIndex() const { return layer_index_; }

  /// Access the layer subsampling factor.
  /// @return The layer subsampling factor.
  inline unsigned subsampling() const { return subsampling_; }

  /// Access the layer flags.
  /// @return The layer @c Flag values.
  inline unsigned flags() const { return flags_; }

  /// Set the layer flags.
  /// @param flags New flags to set.
  inline void setFlags(unsigned flags) { flags_ = flags; }

  /// Copy the @c VoxelLayout from @p other.
  /// @param other Layer to copy the voxel structure of.
  void copyVoxelLayout(const MapLayer &other);

  /// Read only access to the voxel layout of this layer.
  /// @return Voxel layout details.
  VoxelLayoutConst voxelLayout() const;

  /// Writable access to the voxel layout of this layer.
  /// @return Voxel layout configuration.
  VoxelLayout voxelLayout();

  /// Check if this @c MapLayer is equivalent to @p other.
  ///
  /// The layers are may be equivalent if voxel patterns are the same and the clearing patterns match. The names do
  /// not have to be the same.
  ///
  /// The layers match if the names also match.
  ///
  /// @param other The other layer to compare against.
  /// @return The equivalence @c MapLayoutMatch
  MapLayoutMatch checkEquivalent(const MapLayer &other) const;

  /// Gives the per chunk voxel dimensions of this layer given @p regionDim defines the maximum voxel dimensions.
  ///
  /// For each magnitude step of @c subsampling(), each element of @p regionDim is halved to a minimum of 1.
  ///
  /// @param region_dim The dimensions of each chunk the owning @c OccupancyMap.
  /// @return The voxel dimensions for this layer based on @p regionDim.
  inline glm::u8vec3 dimensions(const glm::u8vec3 &region_dim) const
  {
    const glm::u8vec3 dim = (subsampling_ == 0) ? region_dim : region_dim / uint8_t(1u << subsampling_);
    return glm::max(dim, glm::u8vec3(1));
  }

  /// Retrieve the volume of voxels in each chunk for this layer.
  ///
  /// This is the same as the volume of @c dimensions().
  ///
  /// @param region_dim The dimensions of each chunk the owning @c OccupancyMap.
  /// @return The voxel volume for this layer based on @p regionDim.
  inline size_t volume(const glm::u8vec3 &region_dim) const
  {
    const glm::u8vec3 dim = dimensions(region_dim);
    return size_t(dim.x) * size_t(dim.y) * size_t(dim.z);
  }

  /// Query the size of each voxel in this layer in bytes.
  ///
  /// Same as @c voxelLayout()->voxelByteSize().
  ///
  /// @return The size of each voxel in this layer.
  size_t voxelByteSize() const;

  /// Query the byte size this layer in each each @c MapChunk.
  /// @return The size of this layer in each @p MapChunk.
  size_t layerByteSize(const glm::u8vec3 &region_dim) const;

  /// Allocate memory for this layer given @p regionDim.
  /// @param region_dim The dimensions of each chunk the owning @c OccupancyMap.
  /// @return An uninitialised memory block of size @c layerByteSize(regionDim).
  uint8_t *allocate(const glm::u8vec3 &region_dim) const;

  /// Release memory previously allocated by @c allocate().
  /// @param voxels Memory previously allocated by @c allocate().
  static void release(const uint8_t *voxels);

  /// Clear the given layer memory to the default value.
  /// The default values are set by the @c VoxelLayout details.
  /// @param mem The memory to clear, previously allocated from @c allocate().
  /// @param region_dim The dimensions of each region with no subsampling. Subsampling is resolved as needed.
  void clear(uint8_t *mem, const glm::u8vec3 &region_dim) const;

  /// Set the layer index. Used in layer reordering. Must be maintained correctly.
  /// @param index The new layer index.
  inline void setLayerIndex(unsigned index) { layer_index_ = index; }

private:
  std::string name_;
  std::unique_ptr<VoxelLayoutDetail> voxel_layout_;
  uint16_t layer_index_ = 0;
  uint16_t subsampling_ = 0;
  unsigned flags_ = 0;
};
}  // namespace ohm

#endif  // OHM_MAPLAYER_H
