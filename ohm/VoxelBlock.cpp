// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelBlock.h"

#include "MapLayer.h"
#include "VoxelBlockCompressionQueue.h"

#include "private/OccupancyMapDetail.h"

#include <zlib.h>

#include <algorithm>
#include <cstring>

using namespace ohm;

namespace
{
unsigned minimum_buffer_size = 1024u;
int zlib_compression_level = Z_BEST_SPEED;
int zlib_gzip_flag = 0;  // Use 16 to enable GZip.
const int kWindowBits = 14;
const int kZLibMemLevel = 8;
const int kCompressionStrategy = Z_DEFAULT_STRATEGY;

const auto kReleaseDelay = std::chrono::milliseconds(500);
}  // namespace


void VoxelBlock::getCompressionControls(CompressionControls *controls)
{
  controls->minimum_buffer_size = minimum_buffer_size;

  switch (zlib_compression_level)
  {
  default:
  case Z_DEFAULT_COMPRESSION:
    controls->compression_level = kCompressBalanced;
    break;
  case Z_BEST_SPEED:
    controls->compression_level = kCompressFast;
    break;
  case Z_BEST_COMPRESSION:
    controls->compression_level = kCompressMax;
    break;
  }

  if (zlib_gzip_flag)
  {
    controls->compression_type = kCompressGZip;
  }
  else
  {
    controls->compression_type = kCompressDeflate;
  }
}

void VoxelBlock::setCompressionControls(const CompressionControls &controls)
{
  minimum_buffer_size = (controls.minimum_buffer_size > 0) ? controls.minimum_buffer_size : minimum_buffer_size;
  switch (controls.compression_level)
  {
  default:
  case kCompressFast:
    zlib_compression_level = Z_BEST_SPEED;
    break;
  case kCompressBalanced:
    zlib_compression_level = Z_DEFAULT_COMPRESSION;
    break;
  case kCompressMax:
    zlib_compression_level = Z_BEST_COMPRESSION;
    break;
  }

  zlib_gzip_flag = (controls.compression_type == kCompressGZip) ? 16 : 0;
}


VoxelBlock::VoxelBlock(const OccupancyMapDetail *map, const MapLayer &layer)
  : map_(map)
  , layer_index_(layer.layerIndex())
  , uncompressed_byte_size_(layer.layerByteSize(map->region_voxel_dimensions))
{
  initUncompressed(voxel_bytes_, layer);
  flags_ |= kFUncompressed;
  if (needsCompression())
  {
    std::unique_lock<Mutex> guard(access_guard_);
    queueCompression(guard);
  }
}


VoxelBlock::~VoxelBlock()
{}


void VoxelBlock::destroy()
{
  // Don't use scoped lock as we will delete this which would make releasing the lock invalid.
  access_guard_.lock();
  if (flags_ & kFCompressionQueued)
  {
    // Currently queue. Mark for death. The compression queue will destroy it.
    flags_ |= kFMarkedForDeath;
    access_guard_.unlock();
  }
  else
  {
    // Delete now.
    // fprintf(stderr, "0x%" PRIXPTR ", VoxelBlock::destroy()\n", (uintptr_t)this);
    delete this;
  }
}


void VoxelBlock::retain()
{
  std::unique_lock<Mutex> guard(access_guard_);
  ++reference_count_;
  // Ensure uncompressed data are available.
  if (!(flags_ & kFUncompressed))
  {
    std::vector<uint8_t> working_buffer;
    uncompressUnguarded(working_buffer);
    voxel_bytes_.swap(working_buffer);
    flags_ |= kFUncompressed;
  }
}

void VoxelBlock::release()
{
  std::unique_lock<Mutex> guard(access_guard_);
  if (reference_count_ > 0)
  {
    --reference_count_;
    if (needsCompression())
    {
      queueCompression(guard);
    }
  }
}

void VoxelBlock::compressInto(std::vector<uint8_t> &compression_buffer)
{
  std::unique_lock<Mutex> guard(access_guard_);

  // Handle uninitialised buffer. We may not have initialised the buffer yet, but this call requires data to be
  // compressed such as when used for serialisation to disk.
  if (voxel_bytes_.empty())
  {
    initUncompressed(voxel_bytes_, map_->layout.layer(layer_index_));
    flags_ |= kFUncompressed;
  }

  compressUnguarded(compression_buffer);
}

const VoxelBlock::Clock::time_point VoxelBlock::releaseAfter() const
{
  std::unique_lock<Mutex> guard(access_guard_);
  return release_after_;
}

void VoxelBlock::updateLayerIndex(unsigned layer_index)
{
  std::unique_lock<Mutex> guard(access_guard_);
  layer_index_ = layer_index;
}

bool VoxelBlock::needsCompression() const
{
  return reference_count_ == 0 && (flags_ & kFUncompressed) &&
         (map_->flags & MapFlag::kCompressed) == MapFlag::kCompressed;
}

void VoxelBlock::queueCompression(std::unique_lock<Mutex> &guard)
{
  release_after_ = Clock::now() + kReleaseDelay;
  if (!(flags_ & kFCompressionQueued))
  {
    // This flag will be cleared when processed for compression.
    flags_ |= kFCompressionQueued;
    guard.unlock();
    // Add to compression queue.
    VoxelBlockCompressionQueue::instance().push(this);
  }
}

bool VoxelBlock::compressUnguarded(std::vector<uint8_t> &compression_buffer)
{
  if (flags_ & kFUncompressed)
  {
    int ret = Z_OK;
    z_stream stream;
    memset(&stream, 0u, sizeof(stream));
    deflateInit2(&stream, zlib_compression_level, Z_DEFLATED, kWindowBits | zlib_gzip_flag, kZLibMemLevel,
                 kCompressionStrategy);

    stream.next_in = (Bytef *)voxel_bytes_.data();
    stream.avail_in = unsigned(voxel_bytes_.size());

    compression_buffer.reserve(std::max(voxel_bytes_.size() / 10, static_cast<size_t>(minimum_buffer_size)));
    compression_buffer.resize(compression_buffer.capacity());

    stream.avail_out = unsigned(compression_buffer.size());
    stream.next_out = compression_buffer.data();

    int flush_flag = Z_NO_FLUSH;
    do
    {
      ret = deflate(&stream, flush_flag);

      switch (ret)
      {
      case Z_OK:
        // Done with input data. Make sure we change to flushing.
        if (stream.avail_in == 0)
        {
          flush_flag = Z_FINISH;
        }

        // Check for insufficient output data before Z_STREAM_END.
        if (stream.avail_out == 0)
        {
          // Output buffer too small.
          const size_t bytes_so_far = compression_buffer.size();
          compression_buffer.resize(2 * bytes_so_far);
          stream.avail_out = unsigned(compression_buffer.size() - bytes_so_far);
          stream.next_out = compression_buffer.data() + bytes_so_far;
        }
        break;
      case Z_STREAM_END:
        break;
      default:
        // Failed.
        deflateEnd(&stream);
        return false;
      }
    } while (stream.avail_in || ret != Z_STREAM_END);

    // Ensure flush.
    if (flush_flag != Z_FINISH)
    {
      deflate(&stream, Z_FINISH);
    }

    ret = deflateEnd(&stream);
    if (ret != Z_OK)
    {
      return false;
    }

    // Resize compressed buffer.
    compression_buffer.resize(compression_buffer.size() - stream.avail_out);
  }
  else
  {
    // Already compressed. Copy buffer.
    compression_buffer.resize(voxel_bytes_.size());
    if (!voxel_bytes_.empty())
    {
      memcpy(compression_buffer.data(), voxel_bytes_.data(), sizeof(*voxel_bytes_.data()) * voxel_bytes_.size());
    }
  }

  return true;
}

bool VoxelBlock::uncompressUnguarded(std::vector<uint8_t> &expanded_buffer)
{
  if (voxel_bytes_.empty())
  {
    initUncompressed(voxel_bytes_, map_->layout.layer(layer_index_));
    flags_ |= kFUncompressed;
  }

  if (flags_ & kFUncompressed)
  {
    // Simply copy existing bytes.
    expanded_buffer.resize(voxel_bytes_.size());
    if (!voxel_bytes_.empty())
    {
      memcpy(expanded_buffer.data(), voxel_bytes_.data(), sizeof(*voxel_bytes_.data()) * voxel_bytes_.size());
    }
    return true;
  }

  expanded_buffer.resize(uncompressed_byte_size_);

  int ret = Z_OK;
  z_stream stream;
  memset(&stream, 0u, sizeof(stream));
  inflateInit2(&stream, kWindowBits | zlib_gzip_flag);

  stream.avail_in = unsigned(voxel_bytes_.size());
  stream.next_in = voxel_bytes_.data();

  stream.avail_out = unsigned(expanded_buffer.size());
  stream.next_out = static_cast<unsigned char *>(expanded_buffer.data());

  int flush_flag = Z_NO_FLUSH;
  do
  {
    ret = inflate(&stream, flush_flag);

    switch (ret)
    {
    case Z_OK:
      // Check for insufficient output data on flush or before finishing input data. This is an error an error condition
      // as we know how large it should be.
      if (stream.avail_out == 0 && (flush_flag == Z_FINISH || stream.avail_in))
      {
        // Failed.
        inflateEnd(&stream);
        return false;
      }

      // Transition to flush if there is no more input data.
      if (stream.avail_in == 0)
      {
        flush_flag = Z_FINISH;
      }
      break;
    case Z_STREAM_END:
      break;
    default:
      // Failed.
      inflateEnd(&stream);
      return false;
    }
  } while (stream.avail_in || ret != Z_STREAM_END);

  // Ensure flush.
  if (flush_flag != Z_FINISH)
  {
    inflate(&stream, Z_FINISH);
  }

  // Resize compressed buffer.
  expanded_buffer.resize(expanded_buffer.size() - stream.avail_out);
  inflateEnd(&stream);

  return true;
}


void VoxelBlock::initUncompressed(std::vector<uint8_t> &expanded_buffer, const MapLayer &layer)
{
  expanded_buffer.resize(uncompressedByteSize());
  layer.clear(expanded_buffer.data(), map_->region_voxel_dimensions);
}


bool VoxelBlock::setCompressedBytes(const std::vector<uint8_t> &compressed_voxels)
{
  std::unique_lock<Mutex> guard(access_guard_);
  if (reference_count_ == 0)
  {
    voxel_bytes_.resize(compressed_voxels.size());
    if (!compressed_voxels.empty())
    {
      memcpy(voxel_bytes_.data(), compressed_voxels.data(),
             sizeof(*compressed_voxels.data()) * compressed_voxels.size());
    }
    voxel_bytes_.shrink_to_fit();
    // Clear uncompressed flag.
    flags_ &= ~(kFUncompressed | kFCompressionQueued);
    if (flags_ & kFMarkedForDeath)
    {
      // fprintf(stderr, "0x%" PRIXPTR ", VoxelBlock::setCompressedBytes()\n", (uintptr_t)this);
      delete this;
    }
    return true;
  }
  return false;
}
