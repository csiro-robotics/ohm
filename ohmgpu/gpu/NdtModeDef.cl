// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_NDTMODEDEF_H_
#define OHMGPU_NDTMODEDEF_H_

// Disable NDT
#define NDT_NONE 0
// The value to define NDT as when using base NDT occupancy mode.
#define NDT_OM 1
// The value to define NDT as when using base NDT traversability mode.
#define NDT_TM 2

// Ensure NDT is defined (as nothing)
#ifndef NDT
#define NDT NDT_NONE
#endif  // NDT

#endif  // OHMGPU_NDTMODEDEF_H_
