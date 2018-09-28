// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "profile.h"

#include <atomic>
#include <cinttypes>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ohmutil.h"

#ifdef OHM_THREADS
#include <tbb/tbb_thread.h>
#endif // OHM_THREADS

using namespace ohmutil;

namespace ohmutil
{
  struct ProfileRecord
  {
    const char *name;
    const char *parentName;
    ProfileClock::duration totalTime;
    unsigned markerCount;
  };

  struct ProfileScope
  {
    const char *name;
    ProfileClock::time_point startTime;
    ProfileRecord *record;

    inline ProfileScope() {};
    inline ProfileScope(const char *name)
      : name(name)
      , startTime(ProfileClock::now())
    {}

    inline ProfileScope(const char *name, const ProfileClock::time_point &startTime)
      : name(name)
      , startTime(startTime)
    {}

    inline ProfileScope(const char *name, ProfileClock::time_point &&startTime)
      : name(name)
      , startTime(startTime)
    {}

    inline ProfileScope(const ProfileScope &other)
      : name(other.name)
      , startTime(other.startTime)
    {}

    inline ProfileScope(ProfileScope &&other)
      : name(other.name)
      , startTime(std::move(other.startTime))
    {}
  };

  struct ThreadRecords
  {
    std::unordered_map<std::string, ProfileRecord *> records;
    std::vector<ProfileScope> markerStack;

    ~ThreadRecords()
    {
      for (auto &&iter : records)
      {
        delete iter.second;
      }
    }
  };

#ifdef OHM_THREADS
  void compareThreadIds(std::thread::id stdId, tbb::internal::tbb_thread_v3::id tbbId)
  {
    static bool once = true;
    if (once && *(unsigned*)&stdId != *(unsigned*)&tbbId)
    {
      once = false;
    }
  }
#endif // OHM_THREADS

  struct ProfileDetail
  {
    std::mutex mutex;
    std::unordered_map<std::thread::id, ThreadRecords> threadRecords;
    std::atomic_bool reported;
    std::atomic_bool supressReport;

    inline ProfileDetail()
      : reported(true)
      , supressReport(false)
    {}


    inline ThreadRecords &getCurrentThreadRecords()
    {
      std::unique_lock<std::mutex> guard(mutex);
#ifdef OHM_THREADS
      //compareThreadIds(std::this_thread::get_id(), tbb::this_tbb_thread::get_id());
#endif // OHM_THREADS
      auto search = threadRecords.find(std::this_thread::get_id());
      if (search != threadRecords.end())
      {
        return search->second;
      }

      return threadRecords.insert(std::make_pair(std::this_thread::get_id(), ThreadRecords())).first->second;
    }
  };


  void showReport(std::ostream &o, const ProfileRecord &record, const ThreadRecords &threadRecords, int level = 0)
  {
    std::string indent(level * 2, ' ');
    std::string countStr;
    auto averageTime = (record.markerCount) ? record.totalTime / record.markerCount : ProfileClock::duration(0);
    delimetedInteger(countStr, record.markerCount);
    o << indent << record.name << " avg: " << averageTime << " total: " << record.totalTime << " / " << countStr << " calls\n";

    // Recurse on children.
    for (auto &&entry : threadRecords.records)
    {
      if (entry.second->parentName && strcmp(record.name, entry.second->parentName) == 0 &&
          strcmp(record.name, entry.second->name) != 0)
      {
        if (strcmp(record.name, entry.second->name) != 0)
        {
          showReport(o, *entry.second, threadRecords, level + 1);
        }
        else
        {
          o << indent << "*** self reference error ***\n";
        }
      }
    }
  }


  void showReport(std::ostream &o, const std::thread::id &threadId, const ThreadRecords &records)
  {
    o << "thread " << threadId << '\n';
    for (auto &&record : records.records)
    {
      if (record.second->parentName == nullptr)
      {
        // Only display root items, but do so by traversing the children.
        showReport(o, *record.second, records, 1);
      }
    }
  }
}


Profile Profile::s_instance;


Profile::Profile()
  : _imp(new ProfileDetail)
{
}


Profile::~Profile()
{
  report();
  delete _imp;
}


Profile &Profile::instance()
{
  return s_instance;
}


bool Profile::push(const char *name)
{
  _imp->reported = false;
  ThreadRecords &records = _imp->getCurrentThreadRecords();
  if (!records.markerStack.empty())
  {
    if (strcmp(records.markerStack.back().name, name) == 0)
    {
      // Self reference.
      //std::cout << "selfref\n";
      return false;
    }
  }
  records.markerStack.push_back(ProfileScope(name));
  return true;
}


void Profile::pop()
{
  ThreadRecords &records = _imp->getCurrentThreadRecords();
  if (records.markerStack.empty())
  {
    return;
  }

  _imp->reported = false;
  ProfileScope poppedScope = records.markerStack.back();
  auto elapsed = ProfileClock::now() - poppedScope.startTime;
  records.markerStack.pop_back();

  // Key on the parent scope plus the popped scope.
  const char *parentName = (!records.markerStack.empty()) ? records.markerStack.back().name : "";
  std::string key = std::string(parentName) + poppedScope.name;
  auto existingRecord = records.records.find(key);
  ProfileRecord *record = nullptr;
  if (existingRecord != records.records.end())
  {
    record = existingRecord->second;
  }
  else
  {
    ProfileClock::duration zeroDuration(0);
    record = new ProfileRecord{ poppedScope.name, nullptr, zeroDuration, 0u };
    if (!records.markerStack.empty())
    {
      record->parentName = parentName;
    }
    auto insert = records.records.insert(std::make_pair(key, record));
  }

  record->totalTime += elapsed;
  ++record->markerCount;
}


void Profile::report(std::ostream *optr)
{
  //DebugBreak();
  if (!_imp->reported && !_imp->supressReport)
  {
    std::ostream &out = (optr) ? *optr : std::cout;
    std::unique_lock<std::mutex> guard(_imp->mutex);

    std::string countStr;

    out << "----------------------------------------\n";
    out << "Profile report\n";
    out << "----------------------------------------\n";

    for (auto &&records : _imp->threadRecords)
    {
      showReport(out, records.first, records.second);
    }

    out << "----------------------------------------\n";
    out << "----------------------------------------\n";

    _imp->reported = true;
  }
}


void Profile::suppressReport(bool suppress)
{
  _imp->supressReport = suppress;
}


bool Profile::reportSupressed() const
{
  return _imp->supressReport;
}
