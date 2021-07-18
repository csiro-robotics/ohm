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
  None,
  Time = (1 << 0),
  Position = (1 << 1),
  Normals = (1 << 2),
  ColourRgb = (1 << 3),
  ColourAlpha = (1 << 4),
  Intensity = (1 << 5),

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
