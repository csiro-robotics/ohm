//
// @author Kazys Stepanas
//
// Copyright (c) 2015 CSIRO
//
#ifndef PROBABILITY_H_
#define PROBABILITY_H_

#include "ohmconfig.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

namespace ohm
{
  /// Calculate a probability from its log value.
  /// @param value The log probability value.
  /// @return A real probability value [0, 1].
  inline float ohm_API valueToProbability(float value)
  {
    // Ensure -infinity yields a zero probability.
    // Not all environments respect the sign of an infinity. Some will and yield 0 and 1
    // for -infinity and +infinity respectively, other's yield 1. We explicitly correct this.
    return (value == -std::numeric_limits<float>::infinity()) ? 0.0f : 1.0f - (1.0f / (1.0f + exp(value)));
  }

  /// Convert a probability to a storable value. Inverse of @p valueToProbability().
  /// @param probability The probability to convert
  /// @return The value associated with @p probability."ohmconfig.h"
  inline float ohm_API probabilityToValue(float probability)
  {
    return log(probability / (1.0f - probability));
  }
}

#endif  // PROBABILITY_H_
