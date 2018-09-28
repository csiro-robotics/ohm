// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "scopedtimedisplay.h"

#include "ohmutil.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

using namespace ohmutil;

using Clock = std::chrono::high_resolution_clock;

namespace ohmutil
{
  struct ScopedTimeDisplayDetail
  {
    Clock::time_point startTime;
    Clock::time_point endTime;
    ScopedTimeDisplay::CustomLogger logger;
    std::string msgPrefix;
    std::ostream *out = nullptr;
    bool enabled = true;
  };
}

ScopedTimeDisplay::ScopedTimeDisplay(const char *msgPrefix)
  : _imp(new ScopedTimeDisplayDetail)
{
  _imp->msgPrefix = msgPrefix;
  _imp->out = &std::cout;
  _imp->startTime = Clock::now();
}


ScopedTimeDisplay::ScopedTimeDisplay(const char *msgPrefix, std::ostream &out)
  : ScopedTimeDisplay(msgPrefix)
{
  _imp->out = &out;
}


ScopedTimeDisplay::ScopedTimeDisplay(const char *msgPrefix, const CustomLogger &logger)
  : ScopedTimeDisplay(msgPrefix)
{
  _imp->logger = logger;
}


ScopedTimeDisplay::~ScopedTimeDisplay()
{
  finish();
  delete _imp;
}

const char *ScopedTimeDisplay::messagePrefix() const
{
  return _imp ? _imp->msgPrefix.c_str() : nullptr;
}


std::ostream *ScopedTimeDisplay::out() const
{
  return _imp ? _imp->out : nullptr;
}


const ScopedTimeDisplay::CustomLogger &ScopedTimeDisplay::customLogger() const
{
  static ScopedTimeDisplay::CustomLogger empty;
  return _imp ? _imp->logger : empty;
}


void ScopedTimeDisplay::disable()
{
  if (_imp)
  {
    _imp->enabled = false;
  }
}


void ScopedTimeDisplay::finish()
{
  if (_imp)
  {
    if (_imp->enabled)
    {
      _imp->endTime = Clock::now();
      auto duration = _imp->endTime - _imp->startTime;
      if (_imp->logger)
      {
        std::ostringstream ostr;
        ostr << _imp->msgPrefix << ": " << duration << std::endl;
        _imp->logger(ostr.str().c_str());
      }
      else if (_imp->out)
      {
        (*_imp->out) << _imp->msgPrefix << ": " << duration << std::endl;
        ;
      }

      _imp->enabled = false;
    }
  }
}
