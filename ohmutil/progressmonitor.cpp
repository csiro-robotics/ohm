///
/// @file progressmonitor.cpp
/// @author Kazys Stepanas
///
/// Copyright (c) 2014 CSIRO
///
#include "progressmonitor.h"

#include <chrono>


ProgressMonitor::ProgressMonitor(unsigned updateFrequency)
  : _updateFrequency(updateFrequency)
  , _thread(nullptr)
{
  _progress = 0;
  _totalProgress = 0;
  _pass = 0;
  _totalPasses = 0;
  _quit = false;
  _pause = false;
  _paused = false;
  _displayed = false;
  clearDisplayFunction();
}


ProgressMonitor::ProgressMonitor(const DisplayFunction &displayFunc, unsigned updateFrequency)
  : _displayFunc(displayFunc)
  , _updateFrequency(updateFrequency)
  , _thread(nullptr)
{
  _progress = 0;
  _totalProgress = 0;
  _pass = 0;
  _totalPasses = 0;
  _quit = false;
  _pause = false;
  _paused = false;
  _displayed = false;
}


ProgressMonitor::~ProgressMonitor()
{
  quit();
  joinThread();
}


void ProgressMonitor::clearDisplayFunction()
{
  _displayFunc = std::bind(&dummyDisplay, std::placeholders::_1);
}


void ProgressMonitor::startThread(bool paused)
{
  if (_thread)
  {
    return;
  }

  _paused.store(paused);
  _thread = new std::thread(std::bind(&ProgressMonitor::entry, this));
}


void ProgressMonitor::joinThread()
{
  quit();
  if (_thread)
  {
    _thread->join();
    delete _thread;
    _thread = nullptr;
  }
}


void ProgressMonitor::pause()
{
  if (!_pause)
  {
    _pause.store(true);
    while (!_paused);
  }
}


void ProgressMonitor::unpause()
{
  if (_pause)
  {
    _pause.store(false);
  }
}


void ProgressMonitor::incrementProgress()
{
  ++_progress;
}


void ProgressMonitor::incrementProgressBy(uint64_t increment)
{
  _progress += increment;
}


void ProgressMonitor::updateProgress(uint64_t progress)
{
  _progress = progress;
}


void ProgressMonitor::beginProgress(unsigned pass, const Info &info, bool unpause)
{
  _info = info.info;
  _progress = 0;
  _pass = pass;
  _totalProgress = info.total;
  _totalPasses = info.totalPasses;
  _displayed = false;
  if (unpause)
  {
    this->unpause();
  }
}


void ProgressMonitor::endProgress()
{
  pause();

  if (_progress != 0u && _displayFunc)
  {
    Progress prog;
    prog.info.info = _info.c_str();
    prog.info.totalPasses = _totalPasses;
    prog.info.total = _totalProgress;
    prog.pass = _pass;
    prog.progress = _progress;
    _displayed = true;
    _displayFunc(prog);
  }
}


void ProgressMonitor::entry()
{
  const unsigned sleepTimeMs = 1000/_updateFrequency;
  const std::chrono::milliseconds sleepDuration(sleepTimeMs);
  unsigned lastPass = 0u;
  uint64_t lastProgress = 0u;
  while (!_quit)
  {
    if (!_pause)
    {
      unsigned pass = _pass;
      bool displayed = _displayed;
      uint64_t progress = _progress;
      if (!displayed && (progress || _totalProgress) || progress && progress != lastProgress || pass != lastPass)
      {
        _displayed = true;
        Progress prog;
        prog.info.info = _info.c_str();
        prog.info.totalPasses = _totalPasses;
        prog.info.total = _totalProgress;
        prog.pass = pass;
        prog.progress = progress;
        _displayFunc(prog);
      }
      lastProgress = progress;
      lastPass = pass;
    }
    _paused.store(_pause.load());
    std::this_thread::sleep_for(sleepDuration);
  }

  //std::cout << std::endl;
}


void ProgressMonitor::dummyDisplay(const Progress &)
{
  // NOP
}
