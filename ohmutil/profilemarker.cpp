// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "profilemarker.h"

#include "profile.h"

using namespace ohmutil;

ProfileMarker::ProfileMarker(const char *name, bool activate)
 : ProfileMarker(name, nullptr, activate)
{
}


ProfileMarker::ProfileMarker(const char *name, Profile *profile, bool activate)
  : _name(name)
  , _profile(profile)
  , _active(false)
{
  restart(activate);
}


ProfileMarker::~ProfileMarker()
{
  end();
}


void ProfileMarker::end()
{
  if (_active)
  {
    // Pop first so this marker isn't a parent of itself.
    Profile *profile = (_profile == nullptr) ? &Profile::instance() : _profile;
    profile->pop();
    _active = false;
  }
}


void ProfileMarker::restart(bool activate)
{
  if (!_active && activate)
  {
    Profile *profile = (_profile == nullptr) ? &Profile::instance() : _profile;
    _active = profile->push(_name);
  }
}
