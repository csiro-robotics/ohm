// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMMUTEX_H
#define OHMMUTEX_H

#include "OhmConfig.h"

#include <mutex>

#ifdef OHM_FEATURE_THREADS
#include <tbb/spin_mutex.h>
namespace ohm
{
// tbb::mutex has been deprecated in favour of std::mutex
using Mutex = std::mutex;
using SpinMutex = tbb::spin_mutex;
}  // namespace ohm
#else   // OHM_FEATURE_THREADS
namespace ohm
{
using Mutex = std::mutex;
using SpinMutex = std::mutex;
}  // namespace ohm
#endif  // OHM_FEATURE_THREADS

#endif  // OHMMUTEX_H
