// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Points.h"

#include <type_traits>

#define SLAMIO_CHECK_ALIGNMENT(member_c, member_s)                                 \
  static_assert(offsetof(CloudPoint, member_c) == offsetof(SamplePoint, member_s), \
                "CloudPoint/SamplePoint misalignment: " #member_c "/" #member_s)

namespace slamio
{
// Validate assumptions about the CloudPoint/SamplePoint relationship.
static_assert(sizeof(CloudPoint) == sizeof(SamplePoint), "CloudPoint/SamplePoint sizes differ");
// Assert we have POD types so we can use offsetof()
static_assert(std::is_pod<CloudPoint>(), "CloudPoint is not a POD type.");
static_assert(std::is_pod<SamplePoint>(), "SamplePoint is not a POD type.");
SLAMIO_CHECK_ALIGNMENT(timestamp, timestamp);
SLAMIO_CHECK_ALIGNMENT(position, sample);
SLAMIO_CHECK_ALIGNMENT(normal, origin);
SLAMIO_CHECK_ALIGNMENT(colour, colour);
SLAMIO_CHECK_ALIGNMENT(intensity, intensity);
}  // namespace slamio
