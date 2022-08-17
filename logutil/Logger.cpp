// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Logger.h"

#include <iostream>

namespace
{
logutil::LogOStream g_default_logger;
logutil::LogInterface *g_current_logger = &g_default_logger;
}  // namespace

namespace logutil
{
LogInterface::LogInterface(LogLevel level) noexcept
  : level_(level)
{}


LogInterface::~LogInterface() = default;


LogOStream::LogOStream(LogLevel level) noexcept
  : LogInterface(level)
{}


void LogOStream::message(LogLevel level, const char *msg)
{
  if (int(level) <= int(this->level()))
  {
    switch (level)
    {
    default:
    case LogLevel::kTrace:
      std::clog << msg << std::flush;
      break;
    case LogLevel::kInfo:
      std::cout << msg << std::flush;
      break;
    case LogLevel::kWarn:
      std::cout << msg << std::flush;
      break;
    case LogLevel::kError:
    case LogLevel::kFatal:
      std::cerr << msg << std::flush;
      break;
    }
  }
}


LogInterface *logger()
{
  return g_current_logger;
}


LogInterface *setLogger(LogInterface *logger)
{
  auto *previous = g_current_logger;
  g_current_logger = logger;
  return previous;
}


LogInterface *defaultLogger()
{
  return &g_default_logger;
}
}  // namespace logutil
