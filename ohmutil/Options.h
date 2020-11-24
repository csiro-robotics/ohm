// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_OPTIONS_H
#define OHMUTIL_OPTIONS_H

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4267)
#endif  // _MSC_VER

#include "GlmStream.h"

#include <cxxopts/cxxopts.hpp>

template <typename T>
inline std::string optStr(const T &value)
{
  std::ostringstream str;
  str << value;
  return str.str();
}

template <typename T>
inline std::shared_ptr<cxxopts::Value> optVal(T &val)
{
  return cxxopts::value(val)->implicit_value(optStr(val));
}

inline std::shared_ptr<cxxopts::Value> optVal(bool &val)
{
  return cxxopts::value(val)->implicit_value("true")->default_value(val ? "true" : "false");
}

#pragma warning(pop)

#endif  // OHMUTIL_OPTIONS_H
