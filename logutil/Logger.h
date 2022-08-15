// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef LOGUTIL_LOGGER_H
#define LOGUTIL_LOGGER_H

#include <logutil/LogUtilConfig.h>

#include "LoggerDetail.h"

#include <stdexcept>

namespace logutil
{
/// Logging levels
enum class LogLevel : int
{
  /// Fatal error - an exception will be thrown.
  kFatal,
  /// Error message - will attempt to cleanup or continue, but may not succeed.
  kError,
  /// Warning
  kWarn,
  /// Information message
  kInfo,
  /// Debug/trace message.
  kTrace
};

/// Abstract logging interface.
class logutil_API LogInterface
{
public:
  /// Default constructor.
  LogInterface(LogLevel level = LogLevel::kInfo) noexcept;
  /// Virtual destructor.
  virtual ~LogInterface();

  /// Log a message at the given level. Newline characters are not automatically added.
  ///
  /// @note Logging helper functions are allowed to check the log level before calling this function and skip the
  /// call when the current log level is below the @p level argument.
  ///
  /// @param level The severity of the message.
  /// @param msg The message to log. Must include newline characters as desired by the caller.
  virtual void message(LogLevel level, const char *msg) = 0;

  /// Query the current logging level.
  /// @return The current logging level.
  inline LogLevel level() const { return level_; }
  /// Set the target logging level.
  /// @param level The logging  level.
  inline void setLevel(LogLevel level) { level_ = level; }

private:
  LogLevel level_ = LogLevel::kInfo;  ///< Curent logging level.
};

/// Default logging interface, using  @c std::clog , @c std::cout  and @c std::cerr .
///
/// The output stream is selected based on a message @c LogLevel
///
/// | @c LogLevel         | Stream       |
/// | ------------------- | ------------ |
/// | @c LogLevel::kTrace | @c std::clog |
/// | @c LogLevel::kInfo  | @c std::cout |
/// | @c LogLevel::kWarn  | @c std::cout |
/// | @c LogLevel::kError | @c std::cerr |
/// | @c LogLevel::kFatal | @c std::cerr |
class logutil_API LogOStream : public LogInterface
{
public:
  LogOStream(LogLevel level = LogLevel::kInfo) noexcept;

  /// Log a message at the given level. Output stream will be flushed.
  ///
  /// @param level The severity of the message.
  /// @param msg The message to log. Must include newline characters as desired by the caller.
  void message(LogLevel level, const char *msg) override;
};

/// Get the current, global logging output object.
///
/// A @c LogOStream object is installed by default.
///
/// @return The current @c LogInterface .
LogInterface logutil_API *logger();

/// Set the global logging object. Ownership is retained by the caller and a null logger should be set before the
/// @p logger object expires.
///
/// Not threadsafe.
///
/// @param logger A pointer to the logger.
/// @return The previously set logger interface.
LogInterface logutil_API *setLogger(LogInterface *logger);

/// Get the default logger object installed on startup. May be used to restore the @c logger() after a custom logger
/// has been used.
/// @return The default logging object.
LogInterface logutil_API *defaultLogger();

/// A helper for assembling a logger message at the given severity @p level .
///
/// The @p args are converted into a string using @c std::ostringstream however this may change as better string
/// formatting becomes available in C++20.
///
/// @note Argument conversions are affected by some stream modifiers such as:
/// - @c std::boolalpha
///
/// @note No work is done if the @c logger() is null or the @c logger() level is below the selected message severity.
///
/// @param level The message severity.
/// @param args Arguments to convert to string and concatenate into a log message.
template <typename... Args>
inline void message(LogInterface *log_interface, LogLevel level, Args... args)
{
  if (log_interface)
  {
    if (int(log_interface->level()) >= int(level))
    {
      std::ostringstream out;
      logger_detail::prepareStream(out);
      logger_detail::message(out, args...);
      const std::string msg = out.str();
      log_interface->message(level, msg.c_str());
    }
  }
}


/// Log a @c LogLevel::kTrace message using the @c message() function.
/// @param log_iterface The object to logger to.
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void trace(LogInterface *log_interface, Args... args)
{
  message(log_interface, LogLevel::kTrace, args...);
}


/// Log a @c LogLevel::kTrace message using the @c message() function and default @c logger() .
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void trace(Args... args)
{
  message(logger(), LogLevel::kTrace, args...);
}


/// Log a @c LogLevel::kInfo message using the @c message() function.
/// @param log_iterface The object to logger to.
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void info(LogInterface *log_interface, Args... args)
{
  message(log_interface, LogLevel::kInfo, args...);
}


/// Log a @c LogLevel::kInfo message using the @c logger() function and default @c logger() .
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void info(Args... args)
{
  message(logger(), LogLevel::kInfo, args...);
}


/// Log a @c LogLevel::kWarn message using the @c message() function.
/// @param log_iterface The object to logger to.
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void warn(LogInterface *log_interface, Args... args)
{
  message(log_interface, LogLevel::kWarn, args...);
}


/// Log a @c LogLevel::kWarn message using the @c message() function and default @c logger() .
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void warn(Args... args)
{
  message(logger(), LogLevel::kWarn, args...);
}


/// Log a @c LogLevel::kError message using the @c message() function.
/// @param log_iterface The object to logger to.
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void error(LogInterface *log_interface, Args... args)
{
  message(log_interface, LogLevel::kError, args...);
}


/// Log a @c LogLevel::kError message using the @c message() function and default @c logger() .
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void error(Args... args)
{
  message(logger(), LogLevel::kError, args...);
}


/// Log a @c LogLevel::kFatal message using the @c message() function.
///
/// This functions throws a @c std::runtime_error with the assembled message.
///
/// @param log_iterface The object to logger to.
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void fatal(LogInterface *log_interface, Args... args)
{
  std::ostringstream out;
  logger_detail::prepareStream(out);
  logger_detail::message(out, args...);
  const std::string msg = out.str();
  if (log_interface)
  {
    log_interface->message(LogLevel::kFatal, msg.c_str());
  }
  throw std::runtime_error(msg);
}

/// Log a @c LogLevel::kFatal message using the @c message() function and default @c logger() .
///
/// This functions throws a @c std::runtime_error with the assembled message.
///
/// @param args Arguments to convert to string and concatenate into a logger message.
template <typename... Args>
inline void fatal(Args... args)
{
  fatal(logger(), args...);
}
}  // namespace logutil

#endif  // LOGUTIL_LOGGER_H
