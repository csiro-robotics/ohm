//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#include "ProgressMonitor.h"

#include <chrono>


ProgressMonitor::ProgressMonitor(unsigned update_frequency)
  : progress_(0)
  , total_progress_(0)
  , pass_(0)
  , total_passes_(0)
  , quit_(false)
  , pause_(false)
  , paused_(false)
  , displayed_(false)
  , update_frequency_(update_frequency)
{
  clearDisplayFunction();
}


ProgressMonitor::ProgressMonitor(const DisplayFunction &display_func, unsigned update_frequency)
  : ProgressMonitor(update_frequency)
{
  display_func_ = display_func;
  update_frequency_ = update_frequency;
}


ProgressMonitor::~ProgressMonitor()
{
  joinThread();
}


void ProgressMonitor::clearDisplayFunction()
{
  display_func_ = [](const Progress & /*progress*/) {};
}


void ProgressMonitor::startThread(bool paused)
{
  if (thread_)
  {
    return;
  }

  paused_.store(paused);
  thread_ = std::make_unique<std::thread>(std::thread([this]() { this->entry(); }));
}


void ProgressMonitor::joinThread()
{
  quit();
  if (thread_)
  {
    thread_->join();
    thread_.reset(nullptr);
  }
}


void ProgressMonitor::pause()
{
  if (!pause_)
  {
    pause_.store(true);
    while (thread_ && !paused_)
    {
      std::this_thread::yield();
    }
  }
}


void ProgressMonitor::unpause()
{
  if (pause_)
  {
    pause_.store(false);
  }
}


void ProgressMonitor::incrementProgress()
{
  ++progress_;
}


void ProgressMonitor::incrementProgressBy(uint64_t increment)
{
  progress_ += increment;
}


void ProgressMonitor::updateProgress(uint64_t progress)
{
  progress_ = progress;
}


void ProgressMonitor::beginProgress(unsigned pass, const Info &info, bool unpause)
{
  info_ = info.info;
  progress_ = 0;
  pass_ = pass;
  total_progress_ = info.total;
  total_passes_ = info.total_passes;
  displayed_ = false;
  if (unpause)
  {
    this->unpause();
  }
}


void ProgressMonitor::endProgress()
{
  pause();

  if (progress_ != 0u && display_func_)
  {
    Progress prog;
    prog.info.info = info_;
    prog.info.total_passes = total_passes_;
    prog.info.total = total_progress_;
    prog.pass = pass_;
    prog.progress = progress_;
    displayed_ = true;
    display_func_(prog);
  }
}


void ProgressMonitor::entry()
{
  const unsigned sleep_time_ms = 1000 / update_frequency_;
  const std::chrono::milliseconds sleep_duration(sleep_time_ms);
  unsigned last_pass = 0u;
  uint64_t last_progress = 0u;
  while (!quit_)
  {
    if (!pause_)
    {
      unsigned pass = pass_;
      bool displayed = displayed_;
      uint64_t progress = progress_;
      if (!displayed && (progress || total_progress_) || progress && progress != last_progress || pass != last_pass)
      {
        displayed_ = true;
        Progress prog;
        prog.info.info = info_;
        prog.info.total_passes = total_passes_;
        prog.info.total = total_progress_;
        prog.pass = pass;
        prog.progress = progress;
        display_func_(prog);
      }
      last_progress = progress;
      last_pass = pass;
    }
    paused_.store(pause_.load());
    std::this_thread::sleep_for(sleep_duration);
  }

  // std::cout << std::endl;
}
