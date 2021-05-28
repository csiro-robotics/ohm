// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPINMODE_H
#define GPUPINMODE_H

#include "gpuConfig.h"

namespace gputil
{
// Pinning functions.
enum PinMode
{
  /// Null pinning mode - invalid.
  kPinNone = 0,
  /// Pin for reading on CPU.
  kPinRead,
  /// Pin for writing from CPU.
  kPinWrite,
  /// Pin for read/write.
  kPinReadWrite
};
}  // namespace gputil

#endif  // GPUPINMODE_H
