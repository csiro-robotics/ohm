// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelBlockCompressionQueue.h"

#include "VoxelBlock.h"

#include "private/VoxelBlockCompressionQueueDetail.h"

#include <chrono>
#include <cinttypes>

using namespace ohm;

VoxelBlockCompressionQueue &VoxelBlockCompressionQueue::instance()
{
  static VoxelBlockCompressionQueue queue_instance;
  return queue_instance;
}

VoxelBlockCompressionQueue::VoxelBlockCompressionQueue()
  : imp_(new VoxelBlockCompressionQueueDetail)
{}

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
  unsigned ref = --imp_->reference_count;
  if (ref == 0)
  {
    joinCurrentThread();
  }
  else if (ref < 0)
  {
    imp_->reference_count = 0;
  }
}


void VoxelBlockCompressionQueue::push(VoxelBlock *block)
{
  if (imp_->running)
  {
    imp_->compression_queue.push(block);
  }
  else
  {
    // No compression queue exists. Compress immediately.
    std::vector<uint8_t> working_buffer;
    block->compressInto(working_buffer);
    block->setCompressedBytes(working_buffer);
  }
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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const auto time_now = VoxelBlock::Clock::now();
    VoxelBlock *voxels = nullptr;
    while (imp_->compression_queue.try_pop(voxels))
    {
      if (!(voxels->flags_ & VoxelBlock::kFMarkedForDeath))
      {
        if (time_now > voxels->releaseAfter())
        {
          // std::cout << "compress\n" << std::flush;
          // Compress the current item.
          voxels->compressInto(compression_buffer);
          // The following call may fail as the voxels are retained once more. It should get re-added later if needed.
          voxels->setCompressedBytes(compression_buffer);
        }
        else if (!imp_->quit_flag)
        {
          // std::cout << "compress miss\n" << std::flush;
          // Too soon. Push to the back of the list.
          imp_->compression_queue.push(voxels);
          break;
        }
      }
      else
      {
        // Marked for death. Clean it up.
        // Lock access guard to make sure the code that sets the flag has completed
        voxels->access_guard_.lock();
        // fprintf(stderr, "0x%" PRIXPTR ", VoxelBlockCompressionQueue\n", (uintptr_t)voxels);
        delete voxels;
      }
    }
  }
}
