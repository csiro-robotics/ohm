//
// @author Kazys Stepanas
//
// Copyright (c) 2014 CSIRO
//
#ifndef OHMUTIL_PROGRESSMONITOR_H
#define OHMUTIL_PROGRESSMONITOR_H

#include "OhmUtilExport.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <utility>

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
  /// Structure used to provide information about the current activity.
  struct Info
  {
    std::string info;           ///< String label for the current activity.
    unsigned total_passes = 0;  ///< Total number of passes to target. Zero => unknown or irrelevant.
    uint64_t total = 0;         ///< Total number of items to process in each pass. Zero => unknown.

    /// Default constructor.
    Info() {}  // NOLINT(modernize-use-equals-default) Lint(KS): gcc failed to deduce the default code before use

    /// Copy constructor
    /// @param other Object to copy.
    Info(const Info &other) = default;

    /// Constructor specifying the total number of items to process.
    /// @param total The number of items to process.
    explicit Info(uint64_t total)
      : total(total)
    {}
    /// Constructor specifying the number of passes to run and the total number of items to process.
    /// @param passes The number of passes to run.
    /// @param total The number of items to process.
    Info(unsigned passes, uint64_t total)
      : total_passes(passes)
      , total(total)
    {}
    /// Constructor specifying the label and total number of items to process.
    /// @param info Label for the current activity.
    /// @param total The number of items to process.
    Info(std::string info, uint64_t total)
      : info(std::move(info))
      , total(total)
    {}
    /// Constructor specifying the label, number of passes and total number of items to process.
    /// @param info Label for the current activity.
    /// @param passes The number of passes to run.
    /// @param total The number of items to process.
    Info(std::string info, unsigned passes, uint64_t total)
      : info(std::move(info))
      , total_passes(passes)
      , total(total)
    {}
  };

  /// Current progress information.
  struct Progress
  {
    Info info;              ///< Activity details
    unsigned pass = 0;      ///< Number of completed passes
    uint64_t progress = 0;  ///< Number of completed items in the current pass.
  };

  /// Progress callback function.
  using DisplayFunction = std::function<void(const Progress &)>;

  /// Default frequency to call the @c displayFunction() at.
  static const int kDefaultUpdateFrequency = 10;

  /// Constructor.
  /// @param update_frequency Number of calls to make to the @c displayFunction() every second.
  explicit ProgressMonitor(unsigned update_frequency = kDefaultUpdateFrequency);
  /// Constructor.
  /// @param display_func Number of calls to make to the @p display_func every second.
  /// @param update_frequency Number of calls to make to the @c displayFunction() every second.
  explicit ProgressMonitor(const DisplayFunction &display_func, unsigned update_frequency = kDefaultUpdateFrequency);
  /// Destructor - stops and joins the thread.
  ~ProgressMonitor();

  /// Set the display function to call on evey cycle. This is called from the background thread.
  /// @param display_func Display function callback.
  void setDisplayFunction(const DisplayFunction &display_func);
  /// Clear the @c displayFunction() .
  void clearDisplayFunction();
  /// Query the function called on every cycle.
  /// @return The display function callback.
  const DisplayFunction &displayFunction() const;

  /// Start the background display thread.
  /// @param paused True to start in a paused state.
  void startThread(bool paused = false);
  /// Requests a quit then joins the background thread.
  void joinThread();

  /// Pause displaying the progress value. The thread continues to run.
  /// Blocks until pausing is certain to allow display output to finish.
  /// This may wait up to approximately the update frequency interval.
  void pause();

  /// Unpauses display. This does not block.
  void unpause();

  /// Request the background thread quit.
  inline void quit() { quit_.store(true); }

  /// Returns true if the display function has been called since the first call to
  /// @c startThread() or the last call to @c beginProgress().
  /// @return @c true if the display function has been invoked.
  inline bool hasDisplayed() const { return bool(displayed_); }

  /// Increment progress in the current pass by 1. Threadsafe.
  void incrementProgress();
  /// Increment progress in the current pass by @c increment. Threadsafe.
  /// @param increment The progress increment adjustment.
  void incrementProgressBy(uint64_t increment);

  /// Updates progress to the given value.
  /// @param progress The new progress value to set.
  void updateProgress(uint64_t progress);

  /// Begins progress reporting, resetting the current progress value to zero. Also sets the
  /// total progress, defaulting to zero (unknown). Optionally unpauses the progress thread.
  /// @param info Details about the progress to report.
  /// @param unpause @c true to resume the progress thread.
  inline void beginProgress(const Info &info = Info(), bool unpause = true) { beginProgress(0, info, unpause); }
  /// Begin a new pass.
  /// @param pass The starting pass number.
  /// @param info Details about the progress to report.
  /// @param unpause @c true to resume the progress thread.
  void beginProgress(unsigned pass, const Info &info = Info(), bool unpause = true);

  /// Ends reporting of the current progress. The display is first paused by
  /// invoking @c pause(), then if the total is non-zero the display function
  /// is invoked with the progress being equal to the total.
  void endProgress();

protected:
  /// Thread entry point.
  void entry();

private:
  std::string info_;
  std::atomic<uint64_t> progress_;
  std::atomic<uint64_t> total_progress_;
  std::atomic_uint pass_;
  std::atomic_uint total_passes_;
  std::atomic_bool quit_;
  std::atomic_bool pause_;
  std::atomic_bool paused_;
  std::atomic_bool displayed_;
  DisplayFunction display_func_;
  unsigned update_frequency_;
  std::unique_ptr<std::thread> thread_;
};


inline void ProgressMonitor::setDisplayFunction(const DisplayFunction &display_func)
{
  display_func_ = display_func;
}


inline const ProgressMonitor::DisplayFunction &ProgressMonitor::displayFunction() const
{
  return display_func_;
}

#endif  // OHMUTIL_PROGRESSMONITOR_H
