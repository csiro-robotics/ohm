// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <csignal>
#include <iostream>
#include <locale>

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

template <typename OhmPop, typename DataSource>
int ohmpopMain(int argc, char *argv[])
{
  int exit_code = 0;
  OhmPop populator(std::make_shared<DataSource>());

  g_quit = populator.quitLevelPtr();
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  std::cout.imbue(std::locale(""));

  exit_code = populator.parseCommandLineOptions(argc, (const char *const *)argv);
  if (exit_code)
  {
    return exit_code;
  }

  exit_code = populator.run();
  return exit_code;
}
