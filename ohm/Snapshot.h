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

  /// Extract a simplified snapshot of a 3D occupancy map.
  ///

  struct SnapshotRegion
  {
    double time;
    glm::i16vec3 region_key;
    std::vector<uint8_t> data;
  };

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

    struct SnapshotNode
    {
    public:
      SnapshotNode();
      // Construct tree for given map chunk
      SnapshotNode(const MapChunk &ch, SnapshotSrcVoxel &voxel, const OccupancyMap &map);
      // Construct tree from SnapshotRegion (recursively, starting from specified byte/bit, initialise to zero)
      SnapshotNode(const SnapshotRegion &region, size_t &byte_address, uint8_t &bit_address);
      // Note: copies state, does not copy children (required for vector reserve, should not be used otherwise)
      SnapshotNode(const SnapshotNode &ss);
      ~SnapshotNode();

      // Returns the number of free, occupied and unknown voxels underneath node
      std::tuple<uint16_t, uint16_t, uint16_t> simplify(float voxel_size);

      // Layer 0 = root (layer \in (0,..,treeHeight()-1))
      void getVoxels(size_t layer, float voxel_size, glm::f32vec3 centre,
                     std::vector<std::pair<uint8_t, glm::f32vec3>> &voxels) const;

      // Tree height ()
      size_t treeHeight() const;

      // Return approximate memory use
      size_t memoryUse() const;

      // Return total number of nodes (packed memory use is number of nodes divided by four)
      size_t numNode() const;

      // Return snapshot region struct with given time and region key, populating data with map contained in this node
      SnapshotRegion getRegion(double time, const glm::i16vec3 &region_key) const;

      SnapshotNode &operator=(SnapshotNode &&s);

    private:
      /// Recursion for snapshot of part of region. Valid indices are min_ext.x .. max_ext.x-1, etc
      SnapshotNode(const glm::i16vec3 &region_key, SnapshotSrcVoxel &voxel, glm::u8vec3 min_ext,
                   glm::u8vec3 max_ext);

      // Recursion for populating SnapshotRegion with data beneath this node in tree
      // bits_used_in_final_byte contains number of bits used so far in final byte of region.data (if zero, byte has not
      // yet been allocated). Note that first node is represented in LSB.
      void getRegion(SnapshotRegion &region, uint8_t &bits_used_in_final_byte) const;

    private:
      State state_;
      std::unique_ptr<std::array<SnapshotNode, 8>> children_;
      static constexpr uint8_t kStateMask = 0x3; // mask to current node off byte
      static constexpr uint8_t kStateBits = 2; // number of bits (must have even number of these in a byte)
    };

    /// Construct a default initialised snapshot object.
    Snapshot();

    /// Destructor.
    ~Snapshot();

    /// Generate the snapshot around a reference position. 
    ///
    /// @param cull_to The box (xy) to restrict snapshot to.
    /// @return a vector of chunk centres and mean occupancy.
    std::vector<SnapshotRegion> buildSnapshot(const ohm::Aabb &cull_to = ohm::Aabb(0.0));

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
