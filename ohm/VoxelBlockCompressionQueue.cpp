// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelBlockCompressionQueue.h"

#include "VoxelBlock.h"

#include "private/VoxelBlockCompressionQueueDetail.h"

#include <algorithm>
#include <chrono>
#include <cinttypes>

namespace ohm
{
const int kSleepIntervalMs = 50;

VoxelBlockCompressionQueue &VoxelBlockCompressionQueue::instance()
{
  static VoxelBlockCompressionQueue queue_instance;
  return queue_instance;
}

VoxelBlockCompressionQueue::VoxelBlockCompressionQueue(bool test_mode)
  : imp_(new VoxelBlockCompressionQueueDetail)
{
  imp_->test_mode = test_mode;
}

VoxelBlockCompressionQueue::~VoxelBlockCompressionQueue()
{
  if (imp_->running)
  {
    std::unique_lock<VoxelBlockCompressionQueueDetail::Mutex> guard(imp_->ref_lock);
    joinCurrentThread();
  }
}

void VoxelBlockCompressionQueue::retain()
{
  std::unique_lock<VoxelBlockCompressionQueueDetail::Mutex> guard(imp_->ref_lock);
  if (++imp_->reference_count == 1)
  {
    // Start the thread.
    imp_->running = true;
    imp_->processing_thread = std::thread([this]() { this->run(); });
  }
}


void VoxelBlockCompressionQueue::release()
{
  std::unique_lock<VoxelBlockCompressionQueueDetail::Mutex> guard(imp_->ref_lock);
  int ref = --imp_->reference_count;
  if (ref == 0)
  {
    joinCurrentThread();
  }
  else if (ref < 0)
  {
    imp_->reference_count = 0;
  }
}


uint64_t VoxelBlockCompressionQueue::highWaterMark() const
{
  return imp_->high_water_mark;
}


void VoxelBlockCompressionQueue::setHighWaterMark(uint64_t mark)
{
  imp_->high_water_mark = mark;
}


uint64_t VoxelBlockCompressionQueue::lowWaterMark() const
{
  return imp_->low_water_mark;
}


void VoxelBlockCompressionQueue::setLowWaterMark(uint64_t mark)
{
  imp_->low_water_mark = mark;
}


uint64_t VoxelBlockCompressionQueue::estimatedAllocationSize() const
{
  return imp_->estimated_allocated_size;
}


void VoxelBlockCompressionQueue::push(VoxelBlock *block)
{
  if (imp_->running || imp_->test_mode)
  {
    block->flags_ |= VoxelBlock::kFManagedForCompression;
    ohm::push(*imp_, block);
  }
}


bool VoxelBlockCompressionQueue::testMode() const
{
  return imp_->test_mode;
}


void VoxelBlockCompressionQueue::__tick(std::vector<uint8_t> &compression_buffer)
{
  // Process any new items added to the compression queue by adding them to the block list.
  {
    VoxelBlock *voxels = nullptr;

    while (ohm::tryPop(*imp_, &voxels))
    {
      imp_->blocks.emplace_back(CompressionEntry{ voxels, 0u });
    }
  }

  // Estimate the current memory usage and release items marked for death.
  uint64_t memory_usage = 0;
  const uint64_t high_water_mark = imp_->high_water_mark;
  const uint64_t low_water_mark = imp_->low_water_mark;
  for (auto iter = imp_->blocks.begin(); iter != imp_->blocks.end();)
  {
    CompressionEntry &entry = *iter;
    // Check if marked for death.
    if (!(entry.voxels->flags_ & VoxelBlock::kFMarkedForDeath))
    {
      // Still alive. Update th entry's allocation size.
      if (entry.voxels->flags_ & VoxelBlock::kFUncompressed)
      {
        entry.allocation_size = entry.voxels->uncompressed_byte_size_;
      }
      else
      {
        entry.allocation_size = entry.voxels->compressed_byte_size_;
      }

      memory_usage += entry.allocation_size;
      ++iter;
    }
    else
    {
      // Marked for death. Remove this entry;
      // Block no longer required. Remove it.
      // Marked for death. Clean it up.
      // Lock access guard to make sure the code that sets the flag has completed
      iter->voxels->access_guard_.lock();
      // fprintf(stderr, "0x%" PRIXPTR ", VoxelBlockCompressionQueue\n", (uintptr_t)iter->voxels);
      delete iter->voxels;
      iter = imp_->blocks.erase(iter);
    }
  }

  // Check if we are over the high water mark and release what we can.
  if (memory_usage >= high_water_mark)
  {
    // Sort all blocks by allocation size.
    // TODO(KS): consider including the queued for compression flag so we push items to be compressed to the front
    // of the sort results.
    // TODO(KS): try using a partial sort.
    std::sort(imp_->blocks.begin(), imp_->blocks.end(), [](const CompressionEntry &a, const CompressionEntry &b) {
      return a.allocation_size >= b.allocation_size;
    });

    auto iter = imp_->blocks.begin();
    // Free until we reach the low water mark.
    while (iter != imp_->blocks.end() && memory_usage >= low_water_mark)
    {
      // Check if marked for death. The status may have changed since we updated the allocation size.
      if (!(iter->voxels->flags_ & VoxelBlock::kFMarkedForDeath))
      {
        // Not maked for death. Check lock flag and compress this item if we can
        if (!(iter->voxels->flags_ & VoxelBlock::kFLocked))
        {
          // Block is not locked.
          // std::cout << "compress\n" << std::flush;
          // Try compress the current item. This could fail as the flag can have changed. On failure, the
          // compressed_size will be zero. We call compressWithTemporaryBuffer() to re-use the compression buffer
          // memory.
          size_t compressed_size = iter->voxels->compressWithTemporaryBuffer(compression_buffer);
          if (compressed_size)
          {
            // Compression succeeded.
            // Adjust memory_usage down in a way which guarantees no underflow. Paranoia.
            memory_usage = (memory_usage > iter->allocation_size) ? memory_usage - iter->allocation_size : 0u;
            memory_usage += compressed_size;
          }
        }

        ++iter;
      }
      else
      {
        // Block no longer required. Remove it.
        // Marked for death. Clean it up.
        // Lock access guard to make sure the code that sets the flag has completed
        iter->voxels->access_guard_.lock();
        // fprintf(stderr, "0x%" PRIXPTR ", VoxelBlockCompressionQueue\n", (uintptr_t)iter->voxels);
        delete iter->voxels;
        // Adjust memory_usage down in a way which guarantees no underflow. Paranoia.
        memory_usage = (memory_usage > iter->allocation_size) ? memory_usage - iter->allocation_size : 0u;
        iter = imp_->blocks.erase(iter);
      }
    }
  }

  imp_->estimated_allocated_size = memory_usage;
}

void VoxelBlockCompressionQueue::joinCurrentThread()
{
  // Mark thread for quit.
  if (imp_->running)
  {
    imp_->quit_flag = true;
    imp_->processing_thread.join();
    // Clear the running and quit flags.
    imp_->running = false;
    imp_->quit_flag = false;
  }
}


void VoxelBlockCompressionQueue::run()
{
  std::vector<uint8_t> compression_buffer;
  while (!imp_->quit_flag)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepIntervalMs));
    __tick(compression_buffer);
  }
}
}  // namespace ohm
