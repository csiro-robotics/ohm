// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SLAMIO_DATACHANNEL_H_
#define SLAMIO_DATACHANNEL_H_

#include "SlamIOConfig.h"

namespace slamio
{
enum class DataChannel : unsigned;
}

slamio::DataChannel operator~(slamio::DataChannel a);
slamio::DataChannel operator|(slamio::DataChannel a, slamio::DataChannel b);
slamio::DataChannel operator|=(slamio::DataChannel &a, slamio::DataChannel b);
slamio::DataChannel operator&(slamio::DataChannel a, slamio::DataChannel b);
slamio::DataChannel operator&=(slamio::DataChannel &a, slamio::DataChannel b);

namespace slamio
{
enum class DataChannel : unsigned
{
  None = 0u,
  Time = (1u << 0u),
  Position = (1u << 1u),
  Normal = (1u << 2u),
  ColourRgb = (1u << 3u),
  ColourAlpha = (1u << 4u),
  Intensity = (1u << 5u),
  ReturnNumber = (1u << 6u),

  Colour = ColourRgb | ColourAlpha
};
}

inline slamio::DataChannel operator~(slamio::DataChannel a)
{
  return slamio::DataChannel(~unsigned(a));
}

inline slamio::DataChannel operator|(slamio::DataChannel a, slamio::DataChannel b)
{
  return slamio::DataChannel(unsigned(a) | unsigned(b));
}

inline slamio::DataChannel operator|=(slamio::DataChannel &a, slamio::DataChannel b)
{
  a = a | b;
  return a;
}

inline slamio::DataChannel operator&(slamio::DataChannel a, slamio::DataChannel b)
{
  return slamio::DataChannel(unsigned(a) & unsigned(b));
}

inline slamio::DataChannel operator&=(slamio::DataChannel &a, slamio::DataChannel b)
{
  a = a & b;
  return a;
}

#endif  // SLAMIO_DATACHANNEL_H_
