// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMUTEX_H
#define OHMMUTEX_H

#include "OhmConfig.h"

#include <mutex>

#ifdef OHM_THREADS
#include <tbb/mutex.h>
#include <tbb/spin_mutex.h>
namespace ohm
{
  using Mutex = tbb::mutex;
  using SpinMutex = tbb::spin_mutex;
}  // namespace ohm
#else   // OHM_THREADS
namespace ohm
{
  using Mutex = std::mutex;
  using SpinMutex = std::mutex;
}  // namespace ohm
#endif  // OHM_THREADS

#endif  // OHMMUTEX_H
