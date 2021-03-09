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
#include "VoxelBuffer.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

namespace ohm
{
namespace compare
{
namespace
{
template <typename T>
void buildLogMessage(std::ostream &out, T val)
{
  out << val;
}

template <typename T, typename... Args>
void buildLogMessage(std::ostream &out, T val, const Args &...args)
{
  out << val;
  buildLogMessage(out, args...);
}

template <typename... Args>
void logMessage(Log log, Severity severity, const Args &...args)
{
  std::ostringstream msg;
  buildLogMessage(msg, args...);
  log(severity, msg.str());
}

template <typename T, typename... Args>
bool compareItem(const T &val, const T &ref, Log log, Severity severity, const Args &...args)
{
  if (val != ref)
  {
    logMessage(log, severity, args...);
    return false;
  }

  return true;
}

template <typename T>
bool compareDatum(const void *val_ptr, const void *ref_ptr, uint64_t tolerance)
{
  T val{};
  T ref{};
  T epsilon{};
  memcpy(&val, val_ptr, sizeof(T));
  memcpy(&ref, ref_ptr, sizeof(T));
  memcpy(&epsilon, &tolerance, sizeof(T));

  // We may have unsigned quantities, so ensure the we subtract from the larger
  if (ref > val)
  {
    std::swap(ref, val);
  }
  // We do an equality comparison as well as a diff because we may be comparing floating point inf.
  return val == ref || (val - ref) <= epsilon;
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
}


template <typename T>
std::string memberValueErrorString(const void *val_ptr, const void *ref_ptr)
{
  T val{};
  T ref{};
  memcpy(&val, val_ptr, sizeof(T));
  memcpy(&ref, ref_ptr, sizeof(T));

  std::ostringstream str;
  str << "have " << val << " expect " << ref;
  return str.str();
}


std::string memberValueErrorString(DataType::Type data_type, const void *val_ptr, const void *ref_ptr)
{
  switch (data_type)
  {
  case DataType::kInt8:
    return memberValueErrorString<int8_t>(val_ptr, ref_ptr);
  case DataType::kUInt8:
    return memberValueErrorString<uint8_t>(val_ptr, ref_ptr);
  case DataType::kInt16:
    return memberValueErrorString<int16_t>(val_ptr, ref_ptr);
  case DataType::kUInt16:
    return memberValueErrorString<uint16_t>(val_ptr, ref_ptr);
  case DataType::kInt32:
    return memberValueErrorString<int32_t>(val_ptr, ref_ptr);
  case DataType::kUInt32:
    return memberValueErrorString<uint32_t>(val_ptr, ref_ptr);
  case DataType::kInt64:
    return memberValueErrorString<int64_t>(val_ptr, ref_ptr);
  case DataType::kUInt64:
    return memberValueErrorString<uint64_t>(val_ptr, ref_ptr);
  case DataType::kFloat:
    return memberValueErrorString<float>(val_ptr, ref_ptr);
  case DataType::kDouble:
    return memberValueErrorString<double>(val_ptr, ref_ptr);
  default:
    break;
  }

  return "<value-error>";
}


bool compareVoxel(const Key &key, VoxelBuffer<const VoxelBlock> &eval_buffer, VoxelLayoutConst &eval_voxel_layout,
                  VoxelBuffer<const VoxelBlock> &ref_buffer, VoxelLayoutConst &ref_voxel_layout,
                  const MapLayer *tolerance, Log log)
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
    int tolerance_index = (tolerance) ? tolerance->voxelLayout().indexOf(ref_name.c_str()) : -1;

    bool value_match = true;
    if (tolerance_index != -1)
    {
      // Comparison with tolerance.
      switch (ref_voxel_layout.memberType(i))
      {
      case DataType::kInt8:
        value_match = compareDatum<int8_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                           ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                           tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kUInt8:
        value_match = compareDatum<uint8_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                            ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                            tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kInt16:
        value_match = compareDatum<int16_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                            ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                            tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kUInt16:
        value_match = compareDatum<uint16_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                             ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                             tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kInt32:
        value_match = compareDatum<int32_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                            ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                            tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kUInt32:
        value_match = compareDatum<uint32_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                             ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                             tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kInt64:
        value_match = compareDatum<int64_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                            ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                            tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kUInt64:
        value_match = compareDatum<uint64_t>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                             ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                             tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kFloat:
        value_match = compareDatum<float>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                          ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                          tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      case DataType::kDouble:
        value_match = compareDatum<double>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
                                           ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()),
                                           tolerance->voxelLayout().memberClearValue(tolerance_index));
        break;
      default:
        log(Severity::kError, "Unsupported data tolerance");
        break;
      }
    }
    else
    {
      // No tolerance. Raw byte comparison.
      const uint8_t *ref_mem = static_cast<const uint8_t *>(ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()));
      const uint8_t *eval_mem =
        static_cast<const uint8_t *>(eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()));

      for (size_t b = 0; b < ref_voxel_layout.memberSize(i); ++b)
      {
        value_match = value_match && ref_mem[b] == eval_mem[b];
      }
    }

    if (!value_match)
    {
      ok = false;
      std::string error_str = memberValueErrorString(
        ref_voxel_layout.memberType(i), eval_voxel_layout.memberPtr(eval_member_index, eval_buffer.voxelMemory()),
        ref_voxel_layout.memberPtr(i, ref_buffer.voxelMemory()));
      logMessage(log, Severity::kError, "Voxel ", key, " value mismatch on member ", ref_name, ": ", error_str);
    }
  }

  return ok;
}

VoxelsResult compareVoxels(const OccupancyMap &eval_map, const OccupancyMap &ref_map, const std::string &layer_name,
                           const MapLayer *tolerance, unsigned flags, Log log)
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

    if (compareVoxel(key, eval_buffer, eval_voxel_layout, ref_buffer, ref_voxel_layout, tolerance, log))
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


void configureTolerance(ohm::MapLayer &layer, const char *member_name, DataType::Type data_type, uint64_t epsilon)
{
  auto voxel_layout = layer.voxelLayout();
  voxel_layout.addMember(member_name, data_type, epsilon);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, int8_t epsilon)
{
  const uint64_t e = std::abs(epsilon);
  configureTolerance(layer, member_name, DataType::kInt8, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, uint8_t epsilon)
{
  const uint64_t e = epsilon;
  configureTolerance(layer, member_name, DataType::kUInt8, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, int16_t epsilon)
{
  const uint64_t e = std::abs(epsilon);
  configureTolerance(layer, member_name, DataType::kInt16, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, uint16_t epsilon)
{
  const uint64_t e = epsilon;
  configureTolerance(layer, member_name, DataType::kUInt16, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, int32_t epsilon)
{
  const uint64_t e = std::abs(epsilon);
  configureTolerance(layer, member_name, DataType::kInt32, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, uint32_t epsilon)
{
  const uint64_t e = epsilon;
  configureTolerance(layer, member_name, DataType::kUInt32, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, int64_t epsilon)
{
  const uint64_t e = std::abs(epsilon);
  configureTolerance(layer, member_name, DataType::kInt64, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, uint64_t epsilon)
{
  const uint64_t e = epsilon;
  configureTolerance(layer, member_name, DataType::kUInt64, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, float epsilon)
{
  epsilon = std::abs(epsilon);
  uint64_t e = 0;
  static_assert(sizeof(epsilon) <= sizeof(e), "epsilon data type size too large");
  memcpy(&e, &epsilon, sizeof(epsilon));
  configureTolerance(layer, member_name, DataType::kFloat, e);
}


void configureTolerance(ohm::MapLayer &layer, const char *member_name, double epsilon)
{
  epsilon = std::abs(epsilon);
  uint64_t e = 0;
  static_assert(sizeof(epsilon) <= sizeof(e), "epsilon data type size too large");
  memcpy(&e, &epsilon, sizeof(epsilon));
  configureTolerance(layer, member_name, DataType::kDouble, e);
}
}  // namespace compare
}  // namespace ohm
