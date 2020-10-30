// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_SCOPEDTIMEDISPLAY_H
#define OHMUTIL_SCOPEDTIMEDISPLAY_H

#include <functional>
#include <iosfwd>

namespace ohm
{
struct ScopedTimeDisplayDetail;

/// A scope based timer. Displays the time taken in the current scope on destruction.
///
/// Simple usage:
/// @code
/// int main()
/// {
///   ScopedTimeDisplay timer("Execution time");
///
///   // Explicit logging call because this example will have the logger and logExecTime go out of scope too.
///   timer.finish();
/// }
///
/// Usage:
/// @code
/// int main()
/// {
///   Logger logger; // Custom logging system.
///   const auto logExecTime = [&logger] (const char *timingString)
///   {
///     logger.info(timingString);
///   }
///   ScopedTimeDisplay timer("Execution time", logExecTime);
///
///   // Explicit logging call because this example will have the logger and logExecTime go out of scope too.
///   timer.finish();
/// }
/// @endcode
///
/// The default output format is: <tt>&lt;messagePrefix&gt;: &lt;time&gt;\\n</tt>.
class ScopedTimeDisplay
{
public:
  /// Custom logging function signature.
  using CustomLogger = std::function<void(const char *)>;

  /// Create time display with the given message prefix.
  /// @param msg_prefix Output message prefix.
  ScopedTimeDisplay(const char *msg_prefix);
  /// Create time display with the given message prefix, logging to the given output stream.
  /// @param msg_prefix Output message prefix.
  /// @param out Output stream to log to. Must remain valid until after logging.
  ScopedTimeDisplay(const char *msg_prefix, std::ostream &out);
  /// Create time display with the given message prefix and custom logging.
  /// @param msg_prefix Output message prefix.
  /// @param logger Custom log handler. Called with the formatted output message.
  ScopedTimeDisplay(const char *msg_prefix, const CustomLogger &logger);
  /// Destructor. Calls @c finish() to display time message.
  ~ScopedTimeDisplay();

  /// Prefix for display message.
  /// @return Message prefix.
  const char *messagePrefix() const;
  /// Output stream to log to.
  /// @return Output stream. Null if using @c customLogger().
  std::ostream *out() const;
  /// Custom logging handler.
  /// @return Custom logging function to call.
  const CustomLogger &customLogger() const;

  /// Disable logging. Prevents @c finish() from having any effect.
  void disable();

  /// Finish timing and display the message. Can only ever display once and only if @c disable() has
  /// not been called.
  void finish();

private:
  ScopedTimeDisplayDetail *imp_;
};
}  // namespace ohm

#endif  // OHMUTIL_SCOPEDTIMEDISPLAY_H
