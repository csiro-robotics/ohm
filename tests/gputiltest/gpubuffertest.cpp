// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <gtest/gtest.h>

#include <gpubuffer.h>
#include <gpudevice.h>
#include <gpuevent.h>
#include <gpupinnedbuffer.h>
#include <gpuqueue.h>

#include <chrono>

#include <ohmutil.h>

#include <gpueventdetail.h>

extern gputil::Device g_gpu;

namespace gpubuffertest
{
  typedef std::chrono::high_resolution_clock timing_clock;

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
    static const size_t suffixCount = sizeof(suffixes) / sizeof(suffixes[0]);

    int fraction = 0;
    size_t si = 0;

    if (bytes > 1024u)
    {
      while (bytes > 1024u && si + 1 < suffixCount)
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
    const size_t KiB = 1024u;
    const size_t MiB = 1024u * KiB;
    const size_t GiB = 1024u * MiB;
    const size_t smallBufferSize = 256 * KiB;
    size_t smallBufferCount = 5000;
    size_t largeBufferSize = 0;

    // Resolve how many buffers we can allocate, targeting smallBufferCount.
    gputil::Device &gpu = g_gpu;
    size_t predictedMem = smallBufferSize * smallBufferCount * 2;
    // Allow some additional overhead, dividing the device memory by 2.
    while (predictedMem >= g_gpu.deviceMemory() / 2)
    {
      smallBufferCount /= 2;
      predictedMem = smallBufferSize * smallBufferCount * 2;
    }
    largeBufferSize = smallBufferSize * smallBufferCount;

    std::vector<uint8_t> hostBuffer(smallBufferSize);
    std::vector<uint8_t> testBuffer(smallBufferSize);

    //--------------------------------------------------------------------------
    std::cout << "Running timing test on GPU buffer usage." << std::endl;
    std::cout << smallBufferCount << " buffers of size ";
    logBytes(std::cout, smallBufferSize) << std::endl;
    std::cout << "1 buffer of size ";
    logBytes(std::cout, largeBufferSize) << std::endl;

    for (size_t i = 0; i < hostBuffer.size(); ++i)
    {
      hostBuffer[i] = (uint8_t)(i % 256);
    }
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Allocating host accessible buffers." << std::endl;
    gputil::Queue queue = gpu.createQueue();
    gputil::Event largeBufferEvent;
    std::vector<gputil::Event> largeBufferEvents(smallBufferCount);
    std::vector<gputil::Event> smallBufferEvents(smallBufferCount);

    auto allocStart = timing_clock::now();
    gputil::Buffer largeBuffer(gpu, largeBufferSize, gputil::BF_ReadWriteHost);
    std::vector<gputil::Buffer> smallBuffers(smallBufferCount);

    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i] = gputil::Buffer(gpu, smallBufferSize, gputil::BF_ReadWriteHost);
    }
    auto allocEnd = timing_clock::now();
    std::cout << (allocEnd - allocStart) << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped copy to large buffer in chunks." << std::endl;
    auto mappedUp1Start = timing_clock::now();
    gputil::PinnedBuffer buffer(largeBuffer, gputil::PinWrite);
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      buffer.write(hostBuffer.data(), smallBufferSize, smallBufferSize * i);
    }
    buffer.unpin(&queue, nullptr, &largeBufferEvent);
    queue.flush();
    auto mappedUp1Queued = timing_clock::now();
    largeBufferEvent.wait();
    auto mappedUp1End = timing_clock::now();
    std::cout << mappedUp1End - mappedUp1Start << " total => " << mappedUp1Queued - mappedUp1Start << " queue + "
              << mappedUp1End - mappedUp1Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped copy to small buffers." << std::endl;
    auto mappedUp2Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      gputil::PinnedBuffer buffer(smallBuffers[i], gputil::PinWrite);
      buffer.write(hostBuffer.data(), smallBufferSize);
      buffer.unpin(&queue, nullptr, &smallBufferEvents[i]);
    }
    queue.flush();
    auto mappedUp2Queued = timing_clock::now();
    gputil::Event::wait(smallBufferEvents.data(), smallBufferCount);
    auto mappedUp2End = timing_clock::now();
    std::cout << mappedUp2End - mappedUp2Start << " total => " << mappedUp2Queued - mappedUp2Start << " queue + "
              << mappedUp2End - mappedUp2Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped download large buffer." << std::endl;
    auto mappedDown1Start = timing_clock::now();
    buffer = gputil::PinnedBuffer(largeBuffer, gputil::PinRead);
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      buffer.read(testBuffer.data(), smallBufferSize, smallBufferSize * i);
    }
    buffer.unpin(&queue, nullptr, &largeBufferEvent);
    queue.flush();
    auto mappedDown1Queued = timing_clock::now();
    largeBufferEvent.wait();
    auto mappedDown1End = timing_clock::now();

    for (size_t i = 0; i < testBuffer.size(); ++i)
    {
      EXPECT_EQ(testBuffer[i], hostBuffer[i]);
    }

    std::cout << mappedDown1End - mappedDown1Start << " total => " << mappedDown1Queued - mappedDown1Start
              << " queue + " << mappedDown1End - mappedDown1Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing events." << std::endl;
    auto releaseStart = timing_clock::now();
    largeBufferEvent.release();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      largeBufferEvents[i].release();
      smallBufferEvents[i].release();
    }
    auto releaseEnd = timing_clock::now();
    std::cout << releaseEnd - releaseStart << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Mapped download small buffers." << std::endl;
    auto mappedDown2Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      gputil::PinnedBuffer buffer(smallBuffers[i], gputil::PinRead);
      buffer.read(testBuffer.data(), smallBufferSize);
      buffer.unpin(&queue, nullptr, &smallBufferEvents[i]);
    }
    queue.flush();
    auto mappedDown2Queued = timing_clock::now();
    gputil::Event::wait(smallBufferEvents.data(), smallBufferCount);
    auto mappedDown2End = timing_clock::now();

    for (size_t i = 0; i < testBuffer.size(); ++i)
    {
      EXPECT_EQ(testBuffer[i], hostBuffer[i]);
    }

    std::cout << mappedDown2End - mappedDown2Start << " total => " << mappedDown2Queued - mappedDown2Start
              << " queue + " << mappedDown2End - mappedDown2Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing buffers and events." << std::endl;
    releaseStart = timing_clock::now();
    largeBuffer.release();
    largeBufferEvent.release();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i].release();
      largeBufferEvents[i].release();
      smallBufferEvents[i].release();
      std::cout << "\r" << i + 1 << " / " << smallBufferCount << std::flush;
    }
    std::cout << '\n';
    releaseEnd = timing_clock::now();
    std::cout << releaseEnd - releaseStart << std::endl;
    std::cout << '\n';

    //--------------------------------------------------------------------------
    std::cout << "Allocating device only buffers." << std::endl;
    allocStart = timing_clock::now();
    largeBuffer = gputil::Buffer(gpu, largeBufferSize, gputil::BF_ReadWrite);

    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i] = gputil::Buffer(gpu, smallBufferSize, gputil::BF_ReadWrite);
    }
    allocEnd = timing_clock::now();
    std::cout << (allocEnd - allocStart) << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued copy to large buffer in chunks." << std::endl;
    auto queuedUp1Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      largeBuffer.write(hostBuffer.data(), smallBufferSize, smallBufferSize * i, &queue, nullptr, &largeBufferEvents[i]);
    }
    queue.flush();
    auto queuedUp1Queued = timing_clock::now();
    gputil::Event::wait(largeBufferEvents.data(), smallBufferCount);
    queue.finish();
    auto queuedUp1End = timing_clock::now();
    std::cout << queuedUp1End - queuedUp1Start << " total => " << queuedUp1Queued - queuedUp1Start << " queue + "
              << queuedUp1End - queuedUp1Queued << " wait." << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued copy to small buffers." << std::endl;
    auto queuedUp2Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i].write(hostBuffer.data(), smallBufferSize, 0, &queue, nullptr, &smallBufferEvents[i]);
    }
    queue.flush();
    auto queuedUp2Queued = timing_clock::now();
    gputil::Event::wait(smallBufferEvents.data(), smallBufferCount);
    auto queuedUp2End = timing_clock::now();
    std::cout << queuedUp2End - queuedUp2Start << " total => " << queuedUp2Queued - queuedUp2Start << " queue + "
              << queuedUp2End - queuedUp2Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing events." << std::endl;
    releaseStart = timing_clock::now();
    largeBufferEvent.release();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      largeBufferEvents[i].release();
      smallBufferEvents[i].release();
    }
    releaseEnd = timing_clock::now();
    std::cout << releaseEnd - releaseStart << std::endl;
    std::cout << '\n';
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued download large buffer." << std::endl;
    auto queuedDown1Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      largeBuffer.read(testBuffer.data(), smallBufferSize, smallBufferSize * i, &queue, nullptr, &largeBufferEvents[i]);
    }
    queue.flush();
    auto queuedDown1Queued = timing_clock::now();
    gputil::Event::wait(largeBufferEvents.data(), smallBufferCount);
    auto queuedDown1End = timing_clock::now();

    for (size_t i = 0; i < testBuffer.size(); ++i)
    {
      EXPECT_EQ(testBuffer[i], hostBuffer[i]);
    }

    std::cout << queuedDown1End - queuedDown1Start << " total => " << queuedDown1Queued - queuedDown1Start
              << " queue + " << queuedDown1End - queuedDown1Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Queued download small buffers." << std::endl;
    auto queuedDown2Start = timing_clock::now();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i].read(testBuffer.data(), smallBufferSize, 0, &queue, nullptr, &smallBufferEvents[i]);
    }
    queue.flush();
    auto queuedDown2Queued = timing_clock::now();
    gputil::Event::wait(smallBufferEvents.data(), smallBufferCount);
    auto queuedDown2End = timing_clock::now();

    for (size_t i = 0; i < testBuffer.size(); ++i)
    {
      EXPECT_EQ(testBuffer[i], hostBuffer[i]);
    }

    std::cout << queuedDown2End - queuedDown2Start << " total => " << queuedDown2Queued - queuedDown2Start
              << " queue + " << queuedDown2End - queuedDown2Queued << " wait." << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    std::cout << "Releasing buffers and events." << std::endl;
    releaseStart = timing_clock::now();
    largeBuffer.release();
    largeBufferEvent.release();
    for (size_t i = 0; i < smallBufferCount; ++i)
    {
      smallBuffers[i].release();
      largeBufferEvents[i].release();
      smallBufferEvents[i].release();
      std::cout << "\r" << i + 1 << " / " << smallBufferCount << std::flush;
    }
    std::cout << '\n';
    releaseEnd = timing_clock::now();
    std::cout << releaseEnd - releaseStart << std::endl;
    std::cout << '\n';
    queue.finish();
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    auto logTiming = [largeBufferSize]
                        (const char *name,
                         const timing_clock::duration &up,
                         const timing_clock::duration &down, int width)
    {
      std::ostringstream sstr;
      size_t bytesPerSecond;
      int w;
      auto whitespace = [] (int count)
      {
        while (count-- > 0)
        {
          std::cout << ' ';
        }
      };

      sstr << name << std::flush;
      w = (int)sstr.str().size();
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      sstr << up << std::flush;
      w = (int)sstr.str().size();
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      bytesPerSecond = (size_t)(largeBufferSize / std::chrono::duration_cast<std::chrono::duration<double>>(up).count() + 0.5);
      logBytes(sstr, bytesPerSecond) << "/s" << std::flush;
      w = (int)sstr.str().size();
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      sstr << down << std::flush;
      w = (int)sstr.str().size();
      std::cout << sstr.str();
      whitespace(width - w);
      sstr.str(std::string());

      bytesPerSecond = (size_t)(largeBufferSize / std::chrono::duration_cast<std::chrono::duration<double>>(down).count() + 0.5);
      logBytes(sstr, bytesPerSecond) << "/s" << std::flush;
      // w = (int)sstr.str().size();
      std::cout << sstr.str();
      // whitespace(width - w);
      // sstr.str(std::string());

      std::cout << std::endl;
    };

    std::cout << "Summary\n"
              << "Type          Upload        Up Rate       Download      Down Rate" << std::endl;
    logTiming("Map large", (mappedUp1End - mappedUp1Start), (mappedDown1End - mappedDown1Start), 14);
    logTiming("Map small", (mappedUp2End - mappedUp2Start), (mappedDown2End - mappedDown2Start), 14);
    logTiming("Queue large", (queuedUp1End - queuedUp1Start), (queuedDown1End - queuedDown1Start), 14);
    logTiming("Queue small", (queuedUp2End - queuedUp2Start), (queuedDown2End - queuedDown2Start), 14);
  }


  TEST(GpuBuffer, Ref)
  {
    #if GPUTIL_TYPE == GPUTIL_OPENCL
    gputil::Event event;
    gputil::Device &gpu = g_gpu;
    gputil::Buffer buffer(gpu, 64 * 1024u, gputil::BF_ReadWriteHost);
    gputil::Queue queue = gpu.createQueue();
    std::vector<uint8_t> hostBuffer(buffer.size());

    for (size_t i = 0; i < hostBuffer.size(); ++i)
    {
      hostBuffer[i] = (uint8_t)(i % 256);
    }

    buffer.write(hostBuffer.data(), hostBuffer.size(), 0, &queue, nullptr, &event);

    cl_uint refCount = 0;
    cl_event eventOcl = event.detail()->event;

    clGetEventInfo(eventOcl, CL_EVENT_REFERENCE_COUNT, sizeof(refCount), &refCount, nullptr);
    std::cout << refCount << " : queued" << std::endl;
    // Expected references : event, OpenCL(?)
    EXPECT_GE(refCount, 1u);
    clRetainEvent(eventOcl);
    clGetEventInfo(eventOcl, CL_EVENT_REFERENCE_COUNT, sizeof(refCount), &refCount, nullptr);
    std::cout << refCount << " : +ref" << std::endl;
    // Expected references : event, eventOcl + OpenCL(?)
    EXPECT_GE(refCount, 2u);
    queue.finish();
    clGetEventInfo(eventOcl, CL_EVENT_REFERENCE_COUNT, sizeof(refCount), &refCount, nullptr);
    std::cout << refCount << " : finish" << std::endl;
    // Expected references : event, eventOcl
    EXPECT_EQ(refCount, 2u);
    event.release();
    clGetEventInfo(eventOcl, CL_EVENT_REFERENCE_COUNT, sizeof(refCount), &refCount, nullptr);
    std::cout << refCount << " : release" << std::endl;
    // Expected references : eventOcl
    EXPECT_EQ(refCount, 1u);

    clReleaseEvent(eventOcl);
    #endif // GPUTIL_TYPE == GPUTIL_OPENCL
  }
}
