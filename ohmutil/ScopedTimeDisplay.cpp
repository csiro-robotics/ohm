// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ScopedTimeDisplay.h"

#include "OhmUtil.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

using namespace ohm;

using Clock = std::chrono::high_resolution_clock;

namespace ohm
{
  struct ScopedTimeDisplayDetail
  {
    Clock::time_point start_time;
    Clock::time_point end_time;
    ScopedTimeDisplay::CustomLogger logger;
    std::string msg_prefix;
    std::ostream *out = nullptr;
    bool enabled = true;
  };
}  // namespace ohm

ScopedTimeDisplay::ScopedTimeDisplay(const char *msg_prefix)
  : imp_(new ScopedTimeDisplayDetail)
{
  imp_->msg_prefix = msg_prefix;
  imp_->out = &std::cout;
  imp_->start_time = Clock::now();
}


ScopedTimeDisplay::ScopedTimeDisplay(const char *msg_prefix, std::ostream &out)
  : ScopedTimeDisplay(msg_prefix)
{
  imp_->out = &out;
}


ScopedTimeDisplay::ScopedTimeDisplay(const char *msg_prefix, const CustomLogger &logger)
  : ScopedTimeDisplay(msg_prefix)
{
  imp_->logger = logger;
}


ScopedTimeDisplay::~ScopedTimeDisplay()
{
  finish();
  delete imp_;
}

const char *ScopedTimeDisplay::messagePrefix() const
{
  return imp_ ? imp_->msg_prefix.c_str() : nullptr;
}


std::ostream *ScopedTimeDisplay::out() const
{
  return imp_ ? imp_->out : nullptr;
}


const ScopedTimeDisplay::CustomLogger &ScopedTimeDisplay::customLogger() const
{
  static ScopedTimeDisplay::CustomLogger s_empty;
  return imp_ ? imp_->logger : s_empty;
}


void ScopedTimeDisplay::disable()
{
  if (imp_)
  {
    imp_->enabled = false;
  }
}


void ScopedTimeDisplay::finish()
{
  if (imp_)
  {
    if (imp_->enabled)
    {
      imp_->end_time = Clock::now();
      auto duration = imp_->end_time - imp_->start_time;
      if (imp_->logger)
      {
        std::ostringstream ostr;
        ostr << imp_->msg_prefix << ": " << duration << std::endl;
        imp_->logger(ostr.str().c_str());
      }
      else if (imp_->out)
      {
        (*imp_->out) << imp_->msg_prefix << ": " << duration << std::endl;
        ;
      }

      imp_->enabled = false;
    }
  }
}
