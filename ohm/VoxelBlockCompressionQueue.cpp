// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxelBlockCompressionQueue.h"

#include "VoxelBlock.h"

#include "private/VoxelBlockCompressionQueueDetail.h"

#include <chrono>

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
  std::unique_lock<tbb::mutex> guard(imp_->queue_lock);
  if (imp_->thread_data)
  {
    joinCurrentThread(guard);
  }
}

void VoxelBlockCompressionQueue::retain()
{
  std::unique_lock<tbb::mutex> guard(imp_->queue_lock);
  if (++imp_->reference_count == 1)
  {
    // Start the thread.
    VoxelBlockCompressionThreadInfo *thread_data = new VoxelBlockCompressionThreadInfo;
    imp_->thread_data = thread_data;
    imp_->processing_thread = std::make_shared<std::thread>([this, thread_data]() { this->run(thread_data); });
  }
}


void VoxelBlockCompressionQueue::release()
{
  std::unique_lock<tbb::mutex> guard(imp_->queue_lock);
  if (imp_->reference_count > 0)
  {
    if (--imp_->reference_count == 0)
    {
      joinCurrentThread(guard);
    }
  }
}


void VoxelBlockCompressionQueue::push(VoxelBlock *block)
{
  if (imp_->processing_thread)
  {
    imp_->compression_queue.push(block);
    imp_->queue_notify.notify_all();
  }
  else
  {
    // No compression queue exists. Compress immediately.
    std::vector<uint8_t> working_buffer;
    block->compressInto(working_buffer);
    block->setCompressedBytes(working_buffer);
  }
}


void VoxelBlockCompressionQueue::joinCurrentThread(std::unique_lock<tbb::mutex> &guard)
{
  // Mark thread for quit.
  if (imp_->thread_data)
  {
    imp_->thread_data->quit_flag = true;
    // Clear thread ptr and quit flag.
    std::shared_ptr<std::thread> thread = imp_->processing_thread;
    imp_->processing_thread.reset<std::thread>(nullptr);
    imp_->thread_data = nullptr;
    guard.unlock();
    // Notify for quit and join.
    imp_->queue_notify.notify_all();
    thread->join();
  }
}


void VoxelBlockCompressionQueue::run(VoxelBlockCompressionThreadInfo *thread_data)
{
  while (!thread_data->quit_flag)
  {
    std::unique_lock<tbb::mutex> guard(imp_->queue_lock);
    imp_->queue_notify.wait_for(guard, std::chrono::milliseconds(200));
    guard.unlock();

    const auto time_now = std::chrono::high_resolution_clock::now();
    VoxelBlock *voxels = nullptr;
    while (imp_->compression_queue.try_pop(voxels))
    {
      if (!(voxels->flags_ & VoxelBlock::kFMarkedForDeath))
      {
        if (time_now > voxels->releaseAfter())
        {
          // std::cout << "compress\n" << std::flush;
          // Compress the current item.
          voxels->compressInto(thread_data->compression_buffer);
          // The following call may fail as the voxels are retained once more. It should get re-added later if needed.
          voxels->setCompressedBytes(thread_data->compression_buffer);
        }
        else if (!thread_data->quit_flag)
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

  delete thread_data;
}
