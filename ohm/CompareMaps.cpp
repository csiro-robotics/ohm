// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CompareMaps.h"

#include "KeyStream.h"
#include "MapLayer.h"
#include "MapLayout.h"
#include "OccupancyMap.h"

#include <memory>
#include <sstream>
#include <vector>

namespace ohm::compare
{
namespace
{
template <typename T>
void buildLogMessage(std::ostream &out, T val)
{
  out << val;
}

#include "VoxelBuffer.h"
template <typename T, typename... Args>
void buildLogMessage(std::ostream &out, T val, const Args &... args)
{
  out << val;
  buildLogMessage(out, args...);
}

template <typename... Args>
void logMessage(Log log, Severity severity, const Args &... args)
{
  std::ostringstream msg;
  buildLogMessage(msg, args...);
  log(severity, msg.str());
}

template <typename T, typename... Args>
bool compareItem(const T &val, const T &ref, Log log, Severity severity, const Args &... args)
{
  if (val != ref)
  {
    logMessage(log, severity, args...);
    return false;
  }

  return true;
}
}  // namespace

bool compareLayoutLayer(const OccupancyMap &eval_map, const OccupancyMap &ref_map, const std::string &layer_name,
                        unsigned flags, Log log)
{
  (void)flags;
  const MapLayout &eval_layout = eval_map.layout();
  const MapLayout &ref_layout = ref_map.layout();

  int ref_layer_index = ref_layout.layerIndex(layer_name.c_str());
  int eval_layer_index = eval_layout.layerIndex(layer_name.c_str());

  // Check layer presence and naming.
  bool ok = true;
  if (ref_layer_index == -1)
  {
    logMessage(log, Severity::kError, "Reference map does not have layer ", layer_name);
    ok = false;
  }

  if (eval_layer_index == -1)
  {
    logMessage(log, Severity::kError, "Test map does not have layer ", layer_name);
    ok = false;
  }

  if (!ok)
  {
    // Missing layer(s). Can't continue.
    return false;
  }

  const MapLayer &eval_layer = eval_layout.layer(eval_layer_index);
  const MapLayer &ref_layer = ref_layout.layer(ref_layer_index);

  // Check voxel content.
  auto eval_voxels = eval_layer.voxelLayout();
  auto ref_voxels = ref_layer.voxelLayout();

  ok = compareItem(eval_voxels.voxelByteSize(), ref_voxels.voxelByteSize(), log, Severity::kError,  //
                   "Voxel size mismatch: ", eval_voxels.voxelByteSize(), " expect ", ref_voxels.voxelByteSize()) &&
       ok;

  ok = compareItem(eval_voxels.memberCount(), ref_voxels.memberCount(), log, Severity::kError,  //
                   "Voxel member count mismatch: ", eval_voxels.memberCount(), " expect ", ref_voxels.memberCount()) &&
       ok;

  const size_t member_count = std::min(eval_voxels.memberCount(), ref_voxels.memberCount());

  for (size_t i = 0; i < member_count; ++i)
  {
    ok = compareItem<std::string>(eval_voxels.memberName(i), ref_voxels.memberName(i), log, Severity::kError,  //
                                  "Voxel member name mismatch:(", i, ") ", eval_voxels.memberName(i), " expect ",
                                  ref_voxels.memberName(i)) &&
         ok;

    ok = compareItem(eval_voxels.memberType(i), ref_voxels.memberType(i), log, Severity::kError,  //
                     "Voxel member type mismatch:(", i, ") ", eval_voxels.memberType(i), " expect ",
                     ref_voxels.memberType(i)) &&
         ok;

    ok = compareItem(eval_voxels.memberSize(i), ref_voxels.memberSize(i), log, Severity::kError,  //
                     "Voxel member size mismatch:(", i, ") ", eval_voxels.memberSize(i), " expect ",
                     ref_voxels.memberSize(i)) &&
         ok;
  }

  return ok;
}  // namespace ohm::compare


bool compareVoxel(const Key &key, VoxelBuffer<const VoxelBlock> &eval_buffer, VoxelLayoutConst &eval_voxel_layout,
                  VoxelBuffer<const VoxelBlock> &ref_buffer, VoxelLayoutConst &ref_voxel_layout, Log log)
{
  if (!eval_buffer.isValid())
  {
    return false;
  }

  if (!ref_buffer.isValid())
  {
    return false;
  }

  bool ok = true;
  // Iterate reference members and extract from eval. Note we are not validating mismatches here, only validating the
  // data where we can.
  for (size_t i = 0; i < ref_voxel_layout.memberCount(); ++i)
  {
    std::string ref_name = ref_voxel_layout.memberName(i);

    size_t eval_member_index{};

    // Only search if the member at i is the not the same.
    if (i < eval_voxel_layout.memberCount() && ref_name == eval_voxel_layout.memberName(i))
    {
      eval_member_index = i;
    }
    else
    {
      bool found_member = false;
      // Search by name.
      for (size_t j = 0; j < eval_voxel_layout.memberCount(); ++j)
      {
        if (ref_name == eval_voxel_layout.memberName(j))
        {
          eval_member_index = j;
          found_member = true;
          break;
        }
      }

      if (!found_member)
      {
        return false;
      }
    }

    // Check data type match.
    if (eval_voxel_layout.memberType(eval_member_index) != ref_voxel_layout.memberType(i))
    {
      return false;
    }

    // Check value.
    const uint8_t *ref_mem = static_cast<const uint8_t *>(ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()));
    const uint8_t *eval_mem =
      static_cast<const uint8_t *>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()));

    bool value_match = true;
    for (size_t b = 0; b < ref_voxel_layout.memberSize(i); ++b)
    {
      value_match = value_match && ref_mem[b] == eval_mem[b];
    }

    if (!value_match)
    {
      ok = false;
      logMessage(log, Severity::kError, "Voxel ", key, " value mismatch on member ", ref_name);
    }
  }

  return ok;
}

VoxelsResult compareVoxels(const OccupancyMap &eval_map, const OccupancyMap &ref_map, const std::string &layer_name,
                           unsigned flags, Log log)
{
  VoxelsResult result{};

  result.layout_match = compareLayoutLayer(eval_map, ref_map, layer_name, flags, log);
  if (!result.layout_match)
  {
    // Cannot continue on this failure.
    return result;
  }

  int ref_layer_index = ref_map.layout().layerIndex(layer_name.c_str());
  int eval_layer_index = eval_map.layout().layerIndex(layer_name.c_str());

  // We've compared layers so we know the indices are valid.
  VoxelLayoutConst ref_voxel_layout = ref_map.layout().layer(ref_layer_index).voxelLayout();
  VoxelLayoutConst eval_voxel_layout = eval_map.layout().layer(eval_layer_index).voxelLayout();

  VoxelBuffer<const VoxelBlock> ref_buffer;
  VoxelBuffer<const VoxelBlock> eval_buffer;

  Key last_key = Key(nullptr);
  const MapChunk *ref_chunk = nullptr;
  const MapChunk *eval_chunk = nullptr;
  for (auto key : ref_map)
  {
    if (last_key.regionKey() != key.regionKey())
    {
      // Changed regions. Need to update the voxel buffers.
      ref_chunk = ref_map.region(key.regionKey());
      eval_chunk = eval_map.region(key.regionKey());

      ref_buffer = (ref_chunk) ? VoxelBuffer<const VoxelBlock>(ref_chunk->voxel_blocks[ref_layer_index]) :
                                 VoxelBuffer<const VoxelBlock>();
      eval_buffer = (eval_chunk) ? VoxelBuffer<const VoxelBlock>(eval_chunk->voxel_blocks[eval_layer_index]) :
                                   VoxelBuffer<const VoxelBlock>();
    }

    if (compareVoxel(key, eval_buffer, eval_voxel_layout, ref_buffer, ref_voxel_layout, log))
    {
      ++result.voxels_passed;
    }
    else
    {
      ++result.voxels_failed;
      OHM_CMP_FAIL(flags, result);
    }
    last_key = key;
  }

  return result;
}
}  // namespace ohm::compare
