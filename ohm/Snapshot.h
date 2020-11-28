// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Jason Williams
#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include "OhmConfig.h"

#include "Aabb.h"
#include "HeightmapVoxelType.h"
#include "UpAxis.h"

#include <memory>

#include <glm/fwd.hpp>

#include <vector>

namespace ohm
{
  class Key;
  class MapInfo;
  class OccupancyMap;
  class VoxelConst;
  class SnapshotDetail;
  class MapChunk;
  class SnapshotSrcVoxel;
  class SnapshotVoxelBlock;


  class Snapshot
  {
  public:
    enum State : uint8_t
    {
      kFree = 0,
      kOccupied = 1,
      kUnobserved = 2,
      kNonLeaf = 3
    };

    /// Extract a simplified snapshot of a 3D occupancy map.
    ///
    struct EncodedSnapshotRegion
    {
      uint64_t stamp;
      glm::i16vec3 region_key;
      std::vector<uint8_t> data;
    };

    struct EncodedSnapshot
    {
      std::string frame_id;
      uint64_t stamp;
      glm::f32vec3 origin;
      glm::f32vec3 region_spatial_dimensions;
      std::vector<EncodedSnapshotRegion> regions;
    };

    struct SnapshotNode
    {
    public:
      SnapshotNode();
      /// Recursion for snapshot of part of region. Valid indices are min_ext.x .. max_ext.x-1, etc
      SnapshotNode(const glm::i16vec3 &region_key, SnapshotSrcVoxel &voxel, glm::u8vec3 min_ext, glm::u8vec3 max_ext);
      // Construct tree from SnapshotRegion (recursively, starting from specified byte/bit, initialise to zero)
      SnapshotNode(const EncodedSnapshotRegion &region, size_t &byte_address, uint8_t &bit_address);
      // Deep copy (recursive)
      SnapshotNode(const SnapshotNode &ss);
      ~SnapshotNode();

      // Simplify the chunk, discarding children if the configuration of free/occupied/unobserved is sufficiently
      // uniform for the voxel size 
      // Voxel size denotes the dimension of the cube that the node represents 
      // Returns the number of free, occupied and unknown voxels underneath node
      std::tuple<uint16_t, uint16_t, uint16_t> simplify(float voxel_size);

      // Layer 0 = root (layer \in (0,..,treeHeight()-1))
      void getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                     std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const;

      // Return the tree height 
      size_t treeHeight() const;

      // Return approximate memory use
      size_t memoryUse() const;

      // Return total number of nodes (packed memory use is number of nodes divided by four)
      size_t numNode() const;

      // Recursion for populating SnapshotRegion with data beneath this node in tree
      // bits_used_in_final_byte contains number of bits used so far in final byte of region.data (if zero, byte has not
      // yet been allocated). Note that first node is represented in LSB.
      void getRegion(EncodedSnapshotRegion &region, uint8_t &bits_used_in_final_byte) const;

      // Update tree with newer data in s2 (assuming equivalency of voxel spatial locations)
      // Any voxels that are uncertain in s2 remain unchanged
      // Otherwise voxels are replaced with values in s2
      void update(const SnapshotNode &s2);

      // Update the provided MapChunk using the data in this
      void updateOccupancyMap(const glm::i16vec3 &region_key, SnapshotVoxelBlock &voxel, glm::u8vec3 min_ext,
                              glm::u8vec3 max_ext);

      SnapshotNode &operator=(SnapshotNode &&s);

      SnapshotNode &operator=(const SnapshotNode &s);

      static constexpr uint8_t kStateMask = 0x3;  // mask to current node off byte
      static constexpr uint8_t kStateBits = 2;    // number of bits (must have even number of these in a byte)

    private:
      State state_;
      std::unique_ptr<std::array<SnapshotNode, 8>> children_;
    };


    struct SnapshotRegion
    {
    public:
      SnapshotRegion() {}

      // Construct tree for given map chunk
      SnapshotRegion(const MapChunk &ch, const OccupancyMap &map);

      // Construct tree from SnapshotRegion (recursively, starting from specified byte/bit, initialise to zero)
      SnapshotRegion(const EncodedSnapshotRegion &region, float voxel_size);

      // Note: copies state, does not copy children (required for vector reserve, should not be used otherwise)
      SnapshotRegion(const SnapshotRegion &ss);
      ~SnapshotRegion() = default;

      // Returns the number of free, occupied and unknown voxels underneath node
      void simplify() { root_->simplify(region_size_); }

      // Layer 0 = root (layer \in (0,..,treeHeight()-1))
      void getVoxels(size_t layer, std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const;

      // Return the tree height
      size_t treeHeight() const { return root_->treeHeight(); }

      // Return approximate memory use
      size_t memoryUse() const { return root_->memoryUse(); }

      // Return total number of nodes (packed memory use is number of nodes divided by four)
      size_t numNode() const { return root_->numNode(); }

      // Return snapshot region struct, populating data with map contained in this region
      EncodedSnapshotRegion getRegion() const;

      // Update tree with newer data in s2 (assuming equivalency of voxel spatial locations)
      // Any voxels that are uncertain in s2 remain unchanged
      // Otherwise voxels are replaced with values in s2
      void update(const SnapshotRegion &s2);

      // Update the provided MapChunk using the data in this
      void updateOccupancyMap(ohm::OccupancyMap &map) const;


      SnapshotRegion &operator=(SnapshotRegion &&s) = default;

    private:
      std::unique_ptr<SnapshotNode> root_;
      float region_size_;
      glm::i16vec3 region_key_;
      uint64_t stamp_;
    };

    /// Construct a default initialised snapshot object.
    Snapshot();

    /// Destructor.
    ~Snapshot();

    /// Generate the snapshot around a reference position. 
    ///
    /// @param cull_to The box (xy) to restrict snapshot to.
    /// @return Encoded snapshot structure.
    EncodedSnapshot buildSnapshot(const ohm::Aabb &cull_to = ohm::Aabb(0.0));

    /// Set the (source) occupancy map. Does not take ownership of the pointer so the @p map must persist until 
    /// @c buildSnapshot() is called.
    void setOccupancyMap(OccupancyMap *map);

    /// Access the current source occupancy map.
    OccupancyMap *occupancyMap() const;


  private:
    std::unique_ptr<SnapshotDetail> imp_;
  };
}  // namespace ohm

#endif  // HEIGHTMAP_H
