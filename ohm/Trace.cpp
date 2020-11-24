// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Trace.h"

#include <3esservermacros.h>

void ohm::trace::init(const std::string &file_stream)
{
  (void)file_stream;
  // Initialise TES
  TES_SETTINGS(settings, tes::SF_Compress | tes::SF_Collate);
  // Initialise server info.
  TES_SERVER_INFO(info, tes::XYZ);
  // Create the server. Use tesServer declared globally above.
  TES_SERVER_CREATE(ohm::g_tes, settings, &info);

  // Start the server and wait for the connection monitor to start.
  TES_SERVER_START(ohm::g_tes, tes::ConnectionMonitor::Asynchronous);

  TES_IF(!file_stream.empty()) { TES_LOCAL_FILE_STREAM(ohm::g_tes, file_stream.c_str()); }
  TES_SERVER_START_WAIT(ohm::g_tes, 1000);
}


void ohm::trace::done()
{
  TES_SERVER_UPDATE(ohm::g_tes, 0.0f);
  TES_SERVER_STOP(ohm::g_tes);
}
