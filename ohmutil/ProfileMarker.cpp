// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ProfileMarker.h"

#include "Profile.h"

using namespace ohm;

ProfileMarker::ProfileMarker(const char *name, bool activate)
  : ProfileMarker(name, nullptr, activate)
{}


ProfileMarker::ProfileMarker(const char *name, Profile *profile, bool activate)
  : name_(name)
  , profile_(profile)
  , active_(false)
{
  restart(activate);
}


ProfileMarker::~ProfileMarker()
{
  end();
}


void ProfileMarker::end()
{
  if (active_)
  {
    // Pop first so this marker isn't a parent of itself.
    Profile *profile = (profile_ == nullptr) ? &Profile::instance() : profile_;
    profile->pop();
    active_ = false;
  }
}


void ProfileMarker::restart(bool activate)
{
  if (!active_ && activate)
  {
    Profile *profile = (profile_ == nullptr) ? &Profile::instance() : profile_;
    active_ = profile->push(name_);
  }
}
