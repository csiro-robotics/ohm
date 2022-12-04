// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "Profile.h"

#include "ska/bytell_hash_map.hpp"  // NOLINT

#include <atomic>
#include <cinttypes>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <logutil/LogUtil.h>

#ifdef OHM_FEATURE_THREADS
#include <tbb/tbb_thread.h>
#endif  // OHM_FEATURE_THREADS

namespace ohm
{
struct ProfileRecord
{
  const char *name;
  const char *parent_name;
  ProfileClock::duration total_time;
  ProfileClock::duration recent;
  ProfileClock::duration max_time;
  unsigned marker_count;
};

struct ProfileScope  // NOLINT(cppcoreguidelines-pro-type-member-init)
{
  const char *name;
  ProfileClock::time_point start_time;
  ProfileRecord *record;

  inline ProfileScope() = default;  // NOLINT(cppcoreguidelines-pro-type-member-init)
  inline explicit ProfileScope(const char *name)
    : name(name)
    , start_time(ProfileClock::now())
    , record(nullptr)
  {}

  inline ProfileScope(const char *name, const ProfileClock::time_point &start_time)
    : name(name)
    , start_time(start_time)
    , record(nullptr)
  {}

  inline ProfileScope(const char *name, ProfileClock::time_point &&start_time)
    : name(name)
    , start_time(start_time)
    , record(nullptr)
  {}

  inline ProfileScope(const ProfileScope &other) = default;

  inline ProfileScope(ProfileScope &&other) noexcept
    : name(other.name)
    , start_time(other.start_time)
    , record(other.record)
  {}
};

struct ThreadRecords
{
  ska::bytell_hash_map<std::string, ProfileRecord *> records;
  std::vector<ProfileScope> marker_stack;

  ~ThreadRecords()
  {
    for (auto &&iter : records)
    {
      delete iter.second;
    }
  }
};

#ifdef OHM_FEATURE_THREADS
void compareThreadIds(std::thread::id stdId, tbb::internal::tbb_thread_v3::id tbbId)
{
  static bool once = true;
  if (once && *(unsigned *)&stdId != *(unsigned *)&tbbId)
  {
    once = false;
  }
}
#endif  // OHM_FEATURE_THREADS

struct ProfileDetail
{
  std::mutex mutex;
  std::vector<std::pair<std::thread::id, ThreadRecords>> thread_records;
  std::atomic_bool reported;
  std::atomic_bool suppress_report;

  inline ProfileDetail()
    : reported(true)
    , suppress_report(false)
  {}


  inline ThreadRecords &getCurrentThreadRecords()
  {
    std::unique_lock<std::mutex> guard(mutex);
#ifdef OHM_FEATURE_THREADS
    // compareThreadIds(std::this_thread::get_id(), tbb::this_tbb_thread::get_id());
#endif  // OHM_FEATURE_THREADS
    for (auto &search : thread_records)
    {
      if (search.first == std::this_thread::get_id())
      {
        return search.second;
      }
    }

    thread_records.emplace_back(std::make_pair(std::this_thread::get_id(), ThreadRecords()));
    return thread_records.back().second;
  }
};


void showReport(std::ostream &o, const ProfileRecord &record, const ThreadRecords &thread_records, int level = 0)
{
  std::string indent(level * 2, ' ');
  const auto average_time = (record.marker_count) ? record.total_time / record.marker_count : ProfileClock::duration(0);
  o << indent << record.name << " cur: " << record.recent << " avg: " << average_time << " max: " << record.max_time
    << " total: " << record.total_time << " / " << record.marker_count << " calls\n";

  // Recurse on children.
  for (auto &&entry : thread_records.records)
  {
    if (entry.second->parent_name && strcmp(record.name, entry.second->parent_name) == 0 &&
        strcmp(record.name, entry.second->name) != 0)
    {
      if (strcmp(record.name, entry.second->name) != 0)
      {
        showReport(o, *entry.second, thread_records, level + 1);
      }
      else
      {
        o << indent << "*** self reference error ***\n";
      }
    }
  }
}


void showReport(std::ostream &o, const std::thread::id &thread_id, const ThreadRecords &records)
{
  o << "thread " << thread_id << '\n';
  for (auto &&record : records.records)
  {
    if (record.second->parent_name == nullptr)
    {
      // Only display root items, but do so by traversing the children.
      showReport(o, *record.second, records, 1);
    }
  }
}


Profile::Profile()
  : imp_(std::make_unique<ProfileDetail>())
{}


Profile::~Profile()
{
  report();
}


Profile &Profile::instance()
{
  static Profile s_instance;
  return s_instance;
}


bool Profile::push(const char *name)
{
  imp_->reported = false;
  ThreadRecords &records = imp_->getCurrentThreadRecords();
  if (!records.marker_stack.empty())
  {
    if (strcmp(records.marker_stack.back().name, name) == 0)
    {
      // Self reference.
      // std::cout << "selfref\n";
      return false;
    }
  }
  records.marker_stack.emplace_back(ProfileScope(name));
  return true;
}


void Profile::pop()
{
  ThreadRecords &records = imp_->getCurrentThreadRecords();
  if (records.marker_stack.empty())
  {
    return;
  }

  imp_->reported = false;
  const ProfileScope popped_scope = records.marker_stack.back();
  const auto elapsed = ProfileClock::now() - popped_scope.start_time;
  records.marker_stack.pop_back();

  // Key on the parent scope plus the popped scope.
  const char *parent_name = (!records.marker_stack.empty()) ? records.marker_stack.back().name : "";
  std::string key = std::string(parent_name) + popped_scope.name;
  const auto existing_record = records.records.find(key);
  ProfileRecord *record = nullptr;
  if (existing_record != records.records.end())
  {
    record = existing_record->second;
  }
  else
  {
    const ProfileClock::duration zero_duration(0);
    record = new ProfileRecord{ popped_scope.name, nullptr, zero_duration, zero_duration, zero_duration, 0u };
    if (!records.marker_stack.empty())
    {
      record->parent_name = parent_name;
    }
    records.records.insert(std::make_pair(key, record));
  }

  record->total_time += elapsed;
  record->recent = elapsed;
  record->max_time = std::max(record->max_time, record->recent);
  ++record->marker_count;
}


void Profile::report(std::ostream *optr)
{
  if (!imp_->reported && !imp_->suppress_report)
  {
    std::ostream &out = (optr) ? *optr : std::cout;
    std::unique_lock<std::mutex> guard(imp_->mutex);

    out << "----------------------------------------\n";
    out << "Profile report\n";
    out << "----------------------------------------\n";

    for (auto &&records : imp_->thread_records)
    {
      showReport(out, records.first, records.second);
    }

    out << "----------------------------------------\n";
    out << "----------------------------------------\n";

    imp_->reported = true;
  }
}


void Profile::suppressReport(bool suppress)
{
  imp_->suppress_report = suppress;
}


bool Profile::reportSupressed() const
{
  return imp_->suppress_report;
}
}  // namespace ohm
