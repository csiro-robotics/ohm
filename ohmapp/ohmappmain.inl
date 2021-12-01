// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// Provdes a structure for an application which populates a map using MapHarness
#include "MapHarness.h"

#include <csignal>
#include <iostream>
#include <locale>
#include <memory>

namespace
{
/// Global cache of @c ohmapp::MapHarness::quitLevelPtr() updated in @c onSignal()
unsigned *g_quit = nullptr;

/// Handle @c SIGINT and @c SIGTERM by incrementing @c g_quit
void onSignal(int arg)
{
  if (arg == SIGINT || arg == SIGTERM)
  {
    if (g_quit)
    {
      ++(*g_quit);
    }
  }
}
}  // namespace

/// Main program implementation.
///
/// @param argc Command line argument count.
/// @param argv Command line arguments.
/// @param populator Map generation option.
int ohmappMain(int argc, const char *const *argv, ohmapp::MapHarness &populator)
{
  g_quit = populator.quitLevelPtr();
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  std::cout.imbue(std::locale(""));

  int exit_code = populator.parseCommandLineOptions(argc, argv);
  if (exit_code)
  {
    return exit_code;
  }

  exit_code = populator.run();
  return exit_code;
}

/// Overload using the template type as a @c ohmapp::MapHarness .
/// @param argc Command line argument count.
/// @param argv Command line arguments.
/// @param data_source Data source object.
template <typename OhmPop>
int ohmappMain(int argc, const char *const *argv, std::shared_ptr<ohmapp::DataSource> data_source)
{
  OhmPop populator(data_source);
  return ohmappMain(argc, argv, populator);
}

/// Overload using the template types as a @c ohmapp::MapHarness and @c ohmapp::DataSource .
/// @param argc Command line argument count.
/// @param argv Command line arguments.
template <typename OhmPop, typename DataSource>
int ohmappMain(int argc, const char *const *argv)
{
  return ohmappMain<OhmPop>(argc, argv, std::make_shared<DataSource>());
}
