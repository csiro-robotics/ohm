///
/// @file progressmonitor.h
/// @author Kazys Stepanas
///
/// Copyright (c) 2014 CSIRO
///
#ifndef PROGRESSMONITOR_H
#define PROGRESSMONITOR_H

#include "ohmutilexport.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

/// A utility class for monitoring the progress of a point cloud operations.
///
/// The progress monitor runs in a thread and periodically calls the bound
/// @c displayProgress(unsigned) function from this thread.
///
/// Intended usage is as follows:
/// - Create the progress monitor.
/// - Bind the display function.
/// - Periodically call @c incrementProgress() from processing threads to update progress.
///   This is thread safe and uses atomic operations.
/// Call @c quit() once complete and optionally @c join() once done processing.
class ohmutil_API ProgressMonitor
{
public:
  struct Info
  {
    const char *info;
    unsigned totalPasses;
    uint64_t total;

    Info() : info(""), totalPasses(0), total(0) {}
    Info(uint64_t total) : info(""), totalPasses(0), total(total) {}
    Info(unsigned passes, uint64_t total) : info(""), totalPasses(passes), total(total) {}
    Info(const char *info, uint64_t total) : info(info), totalPasses(0), total(total) {}
    Info(const char *info, unsigned passes, uint64_t total) : info(info), totalPasses(passes), total(total) {}
  };

  struct Progress
  {
    Info info;
    unsigned pass;
    uint64_t progress;
  };

  typedef std::function<void (const Progress &)> DisplayFunction;

  ProgressMonitor(unsigned updateFrequency = 20);
  ProgressMonitor(const DisplayFunction &displayFunc, unsigned updateFrequency = 20);
  ~ProgressMonitor();

  void setDisplayFunction(const DisplayFunction &displayFunc);
  void clearDisplayFunction();
  const DisplayFunction &displayFunction() const;

  void startThread(bool paused = false);
  void joinThread();

  /// Pause displaying the progress value. The thread continues to run.
  /// Blocks until pausing is certain to allow display output to finish.
  /// This may wait up to approximately the update frequency interval.
  void pause();

  /// Unpauses display. This does not block.
  void unpause();

  inline void quit() { _quit.store(true); }

  /// Returns true if the display function has been called since the first call to
  /// @c startThread() or the last call to @c beginProgress().
  /// @return @c true if the display function has been invoked.
  inline bool hasDisplayed() const { return bool(_displayed); }

  void incrementProgress();
  void incrementProgressBy(uint64_t increment);

  /// Updates progress to the given value.
  /// @param progress The new progress value to set.
  void updateProgress(uint64_t progress);

  /// Begins progress reporting, resetting the current progress value to zero. Also sets the
  /// total progress, defaulting to zero (unknown). Optionally unpauses the progress thread.
  /// @param totalProgress The total progress value to report.
  /// @param unpause @c true to resume the progress thread.
  inline void beginProgress(const Info &info = Info(), bool unpause = true) { beginProgress(0, info, unpause); }
  void beginProgress(unsigned pass, const Info &info = Info(), bool unpause = true);

  /// Ends reporting of the current progress. The display is first paused by
  /// invoking @c pause(), then if the total is non-zero the display function
  /// is invoked with the progress being equal to the total.
  void endProgress();

protected:
  void entry();

  static void dummyDisplay(const Progress &);

private:
  std::string _info;
  std::atomic<uint64_t> _progress;
  std::atomic<uint64_t> _totalProgress;
  std::atomic_uint _pass;
  std::atomic_uint _totalPasses;
  std::atomic_bool _quit;
  std::atomic_bool _pause;
  std::atomic_bool _paused;
  std::atomic_bool _displayed;
  DisplayFunction _displayFunc;
  unsigned _updateFrequency;
  std::thread *_thread;
};


inline void ProgressMonitor::setDisplayFunction(const DisplayFunction &displayFunc)
{
  _displayFunc = displayFunc;
}


inline const ProgressMonitor::DisplayFunction &ProgressMonitor::displayFunction() const
{
  return _displayFunc;
}

#endif // PROGRESSMONITOR_H
