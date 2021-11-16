// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapHarness.h"

#include <csignal>
#include <iostream>
#include <locale>
#include <memory>

namespace
{
unsigned *g_quit = nullptr;

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

template <typename OhmPop>
int ohmappMain(int argc, const char *const *argv, std::shared_ptr<ohmapp::DataSource> data_source)
{
  OhmPop populator(data_source);
  return ohmappMain(argc, argv, populator);
}

template <typename OhmPop, typename DataSource>
int ohmappMain(int argc, const char *const *argv)
{
  return ohmappMain<OhmPop>(argc, argv, std::make_shared<DataSource>());
}
