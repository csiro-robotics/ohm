// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuQueue.h>
#include <gputil/cl/gpuEventDetail.h>

#include <chrono>

#include <ohmutil/OhmUtil.h>

extern gputil::Device g_gpu;

namespace gpubuffertest
{
  typedef std::chrono::high_resolution_clock TimingClock;

  const size_t kKiB = 1024u;
  // const size_t kMiB = 1024u * kKiB;
  // const size_t kGiB = 1024u * kMiB;

  std::ostream &logBytes(std::ostream &out, size_t bytes)
  {
    static const char *suffixes[] =
    {
      "byte(s)",
      "KiB",
      "MiB",
      "GiB",
      "TiB",
      "PiB",
      "EiB"
    };
    static const size_t kSuffixCount = sizeof(suffixes) / sizeof(suffixes[0]);

    int fraction = 0;
    size_t si = 0;

    if (bytes > 1024u)
    {
      while (bytes > 1024u && si + 1 < kSuffixCount)
      {
        fraction = int((bytes % 1024u) * 100.0f / 1024.0f + 0.5f);
        bytes /= 1024u;
        ++si;
      }
    }

    if (fraction)
    {
      std::streamsize width = out.width();
      char fill = out.fill();
      out << bytes << "." << std::setw(2) << std::setfill('0') << fraction << suffixes[si];
      out.width(width);
      out.fill(fill);
    }
    else
    {
      out << bytes << suffixes[si];
    }

    return out;
  }

  TEST(GpuBuffer, MemRate)
  {
    // We are going to test the memory transfer rates between host and GPU memory comparing:
    // - Unmapped vs. unmapped transfer.
    // - Synchronous vs. asynchronous transfer.
    // - Many small buffers vs. one large buffer with offsets.
    const size_t small_buffer_size = 256 * kKiB;
    size_t small_buffer_count = 5000;
    size_t large_buffer_size = 0;

    // Resolve how many buffers we can allocate, targeting smallBufferCount.
    gputil::Device &gpu = g_gpu;
    size_t predicted_mem = small_buffer_size * small_buffer_count * 2;
    // Allow some additional overhead, dividing the device memory by 2.
    while (predicted_mem >= g_gpu.deviceMemory() / 2)
    {
      small_buffer_count /= 2;
      predicted_mem = small_buffer_size * small_buffer_count * 2;
    }
    large_buffer_size = small_buffer_size * small_buffer_count;

    std::vector<uint8_t> host_buffer(small_buffer_size);
    std::vector<uint8_t> test_buffer(small_buffer_size);

    //--------------------------------------------------------------------------
    std::cout << "Running timing test on GPU buffer usage." << std::endl;
    std::cout << small_buffer_count << " buffers of size ";
    logBytes(std::cout, small_buffer_size) << std::endl;
    std::cout << "1 buffer of size ";
    logBytes(std::cout, large_buffer_size) << std::endl;

    for (size_t i = 0; i < host_buffer.size(); ++i)
    {
      host_buffer[i] = uint8_t(i % 256);
    }
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Allocating host accessible buffers." << std::endl;
    gputil::Queue queue = gpu.createQueue();
    gputil::Event large_buffer_event;
    std::vector<gputil::Event> large_buffer_events(small_buffer_count);
    std::vector<gputil::Event> small_buffer_events(small_buffer_count);

    auto alloc_start = TimingClock::now();
    gputil::Buffer large_buffer(gpu, large_buffer_size, gputil::kBfReadWriteHost);
    std::vector<gputil::Buffer> small_buffers(small_buffer_count);

    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i] = gputil::Buffer(gpu, small_buffer_size, gputil::kBfReadWriteHost);
    }
    auto alloc_end = TimingClock::now();
    std::cout << (alloc_end - alloc_start) << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped copy to large buffer in chunks." << std::endl;
    const auto mapped_up1_start = TimingClock::now();
    gputil::PinnedBuffer buffer(large_buffer, gputil::kPinWrite);
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      buffer.write(host_buffer.data(), small_buffer_size, small_buffer_size * i);
    }
    buffer.unpin(&queue, nullptr, &large_buffer_event);
    queue.flush();
    const auto mapped_up1_queued = TimingClock::now();
    large_buffer_event.wait();
    const auto mapped_up1_end = TimingClock::now();
    std::cout << mapped_up1_end - mapped_up1_start << " total => " << mapped_up1_queued - mapped_up1_start << " queue + "
              << mapped_up1_end - mapped_up1_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped copy to small buffers." << std::endl;
    const auto mapped_up2_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      gputil::PinnedBuffer buffer(small_buffers[i], gputil::kPinWrite);
      buffer.write(host_buffer.data(), small_buffer_size);
      buffer.unpin(&queue, nullptr, &small_buffer_events[i]);
    }
    queue.flush();
    const auto mapped_up2_queued = TimingClock::now();
    gputil::Event::wait(small_buffer_events.data(), small_buffer_count);
    const auto mapped_up2_end = TimingClock::now();
    std::cout << mapped_up2_end - mapped_up2_start << " total => " << mapped_up2_queued - mapped_up2_start << " queue + "
              << mapped_up2_end - mapped_up2_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped download large buffer." << std::endl;
    const auto mapped_down1_start = TimingClock::now();
    buffer = gputil::PinnedBuffer(large_buffer, gputil::kPinRead);
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      buffer.read(test_buffer.data(), small_buffer_size, small_buffer_size * i);
    }
    buffer.unpin(&queue, nullptr, &large_buffer_event);
    queue.flush();
    const auto mapped_down1_queued = TimingClock::now();
    large_buffer_event.wait();
    const auto mapped_down1_end = TimingClock::now();

    for (size_t i = 0; i < test_buffer.size(); ++i)
    {
      EXPECT_EQ(test_buffer[i], host_buffer[i]);
    }

    std::cout << mapped_down1_end - mapped_down1_start << " total => " << mapped_down1_queued - mapped_down1_start
              << " queue + " << mapped_down1_end - mapped_down1_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing events." << std::endl;
    auto release_start = TimingClock::now();
    large_buffer_event.release();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      large_buffer_events[i].release();
      small_buffer_events[i].release();
    }
    auto release_end = TimingClock::now();
    std::cout << release_end - release_start << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped download small buffers." << std::endl;
    const auto mapped_down2_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      gputil::PinnedBuffer buffer(small_buffers[i], gputil::kPinRead);
      buffer.read(test_buffer.data(), small_buffer_size);
      buffer.unpin(&queue, nullptr, &small_buffer_events[i]);
    }
    queue.flush();
    const auto mapped_down2_queued = TimingClock::now();
    gputil::Event::wait(small_buffer_events.data(), small_buffer_count);
    const auto mapped_down2_end = TimingClock::now();

    for (size_t i = 0; i < test_buffer.size(); ++i)
    {
      EXPECT_EQ(test_buffer[i], host_buffer[i]);
    }

    std::cout << mapped_down2_end - mapped_down2_start << " total => " << mapped_down2_queued - mapped_down2_start
              << " queue + " << mapped_down2_end - mapped_down2_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing buffers and events." << std::endl;
    release_start = TimingClock::now();
    large_buffer.release();
    large_buffer_event.release();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i].release();
      large_buffer_events[i].release();
      small_buffer_events[i].release();
      std::cout << "\r" << i + 1 << " / " << small_buffer_count << std::flush;
    }
    std::cout << '\n';
    release_end = TimingClock::now();
    std::cout << release_end - release_start << std::endl;
    std::cout << '\n';

    //--------------------------------------------------------------------------
    std::cout << "Allocating device only buffers." << std::endl;
    alloc_start = TimingClock::now();
    large_buffer = gputil::Buffer(gpu, large_buffer_size, gputil::kBfReadWrite);

    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i] = gputil::Buffer(gpu, small_buffer_size, gputil::kBfReadWrite);
    }
    alloc_end = TimingClock::now();
    std::cout << (alloc_end - alloc_start) << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued copy to large buffer in chunks." << std::endl;
    const auto queued_up1_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      large_buffer.write(host_buffer.data(), small_buffer_size, small_buffer_size * i, &queue, nullptr, &large_buffer_events[i]);
    }
    queue.flush();
    const auto queued_up1_queued = TimingClock::now();
    gputil::Event::wait(large_buffer_events.data(), small_buffer_count);
    queue.finish();
    const auto queued_up1_end = TimingClock::now();
    std::cout << queued_up1_end - queued_up1_start << " total => " << queued_up1_queued - queued_up1_start << " queue + "
              << queued_up1_end - queued_up1_queued << " wait." << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued copy to small buffers." << std::endl;
    const auto queued_up2_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i].write(host_buffer.data(), small_buffer_size, 0, &queue, nullptr, &small_buffer_events[i]);
    }
    queue.flush();
    const auto queued_up2_queued = TimingClock::now();
    gputil::Event::wait(small_buffer_events.data(), small_buffer_count);
    const auto queued_up2_end = TimingClock::now();
    std::cout << queued_up2_end - queued_up2_start << " total => " << queued_up2_queued - queued_up2_start << " queue + "
              << queued_up2_end - queued_up2_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing events." << std::endl;
    release_start = TimingClock::now();
    large_buffer_event.release();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      large_buffer_events[i].release();
      small_buffer_events[i].release();
    }
    release_end = TimingClock::now();
    std::cout << release_end - release_start << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued download large buffer." << std::endl;
    const auto queued_down1_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      large_buffer.read(test_buffer.data(), small_buffer_size, small_buffer_size * i, &queue, nullptr, &large_buffer_events[i]);
    }
    queue.flush();
    const auto queued_down1_queued = TimingClock::now();
    gputil::Event::wait(large_buffer_events.data(), small_buffer_count);
    const auto queued_down1_end = TimingClock::now();

    for (size_t i = 0; i < test_buffer.size(); ++i)
    {
      EXPECT_EQ(test_buffer[i], host_buffer[i]);
    }

    std::cout << queued_down1_end - queued_down1_start << " total => " << queued_down1_queued - queued_down1_start
              << " queue + " << queued_down1_end - queued_down1_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued download small buffers." << std::endl;
    const auto queued_down2_start = TimingClock::now();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i].read(test_buffer.data(), small_buffer_size, 0, &queue, nullptr, &small_buffer_events[i]);
    }
    queue.flush();
    const auto queued_down2_queued = TimingClock::now();
    gputil::Event::wait(small_buffer_events.data(), small_buffer_count);
    const auto queued_down2_end = TimingClock::now();

    for (size_t i = 0; i < test_buffer.size(); ++i)
    {
      EXPECT_EQ(test_buffer[i], host_buffer[i]);
    }

    std::cout << queued_down2_end - queued_down2_start << " total => " << queued_down2_queued - queued_down2_start
              << " queue + " << queued_down2_end - queued_down2_queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing buffers and events." << std::endl;
    release_start = TimingClock::now();
    large_buffer.release();
    large_buffer_event.release();
    for (size_t i = 0; i < small_buffer_count; ++i)
    {
      small_buffers[i].release();
      large_buffer_events[i].release();
      small_buffer_events[i].release();
      std::cout << "\r" << i + 1 << " / " << small_buffer_count << std::flush;
    }
    std::cout << '\n';
    release_end = TimingClock::now();
    std::cout << release_end - release_start << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    const auto log_timing = [large_buffer_size]
                        (const char *name,
                         const TimingClock::duration &up,
                         const TimingClock::duration &down, int width)
    {
      std::ostringstream sstr;
      size_t bytes_per_second;
      int w;
      auto whitespace = [] (int count)
      {
        while (count-- > 0)
        {
          std::cout << ' ';
        }
      };

      sstr << name << std::flush;
      w = int(sstr.str().size());
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      sstr << up << std::flush;
      w = int(sstr.str().size());
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      bytes_per_second = size_t(large_buffer_size / std::chrono::duration_cast<std::chrono::duration<double>>(up).count() + 0.5);
      logBytes(sstr, bytes_per_second) << "/s" << std::flush;
      w = int(sstr.str().size());
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      sstr << down << std::flush;
      w = int(sstr.str().size());
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      bytes_per_second = size_t(large_buffer_size / std::chrono::duration_cast<std::chrono::duration<double>>(down).count() + 0.5);
      logBytes(sstr, bytes_per_second) << "/s" << std::flush;
      // w = (int)sstr.str().size();
      std::cout << sstr.str();
      // whitespace(width - w);
      // sstr.str(std::string());

      std::cout << std::endl;
    };

    std::cout << "Summary\n"
              << "Type          Upload        Up Rate       Download      Down Rate" << std::endl;
    log_timing("Map large", (mapped_up1_end - mapped_up1_start), (mapped_down1_end - mapped_down1_start), 14);
    log_timing("Map small", (mapped_up2_end - mapped_up2_start), (mapped_down2_end - mapped_down2_start), 14);
    log_timing("Queue large", (queued_up1_end - queued_up1_start), (queued_down1_end - queued_down1_start), 14);
    log_timing("Queue small", (queued_up2_end - queued_up2_start), (queued_down2_end - queued_down2_start), 14);
  }


  TEST(GpuBuffer, Ref)
  {
    #if GPUTIL_TYPE == GPUTIL_OPENCL
    gputil::Event event;
    gputil::Device &gpu = g_gpu;
    gputil::Buffer buffer(gpu, 64 * 1024u, gputil::kBfReadWriteHost);
    gputil::Queue queue = gpu.createQueue();
    std::vector<uint8_t> host_buffer(buffer.size());

    for (size_t i = 0; i < host_buffer.size(); ++i)
    {
      host_buffer[i] = uint8_t(i % 256);
    }

    buffer.write(host_buffer.data(), host_buffer.size(), 0, &queue, nullptr, &event);

    cl_uint ref_count = 0;
    const cl_event event_ocl = event.detail()->event;

    clGetEventInfo(event_ocl, CL_EVENT_REFERENCE_COUNT, sizeof(ref_count), &ref_count, nullptr);
    std::cout << ref_count << " : queued" << std::endl;
    // Expected references : event, OpenCL(?)
    EXPECT_GE(ref_count, 1u);
    clRetainEvent(event_ocl);
    clGetEventInfo(event_ocl, CL_EVENT_REFERENCE_COUNT, sizeof(ref_count), &ref_count, nullptr);
    std::cout << ref_count << " : +ref" << std::endl;
    // Expected references : event, eventOcl + OpenCL(?)
    EXPECT_GE(ref_count, 2u);
    queue.finish();
    clGetEventInfo(event_ocl, CL_EVENT_REFERENCE_COUNT, sizeof(ref_count), &ref_count, nullptr);
    std::cout << ref_count << " : finish" << std::endl;
    // Expected references : event, eventOcl
    EXPECT_EQ(ref_count, 2u);
    event.release();
    clGetEventInfo(event_ocl, CL_EVENT_REFERENCE_COUNT, sizeof(ref_count), &ref_count, nullptr);
    std::cout << ref_count << " : release" << std::endl;
    // Expected references : eventOcl
    EXPECT_EQ(ref_count, 1u);

    clReleaseEvent(event_ocl);
    #endif // GPUTIL_TYPE == GPUTIL_OPENCL
  }
}
