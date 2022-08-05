// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuException.h"
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace gputil
{
#ifdef WIN32
#define strncpy(dst, msg, len) strncpy_s(dst, len + 1, msg, len)
#endif  // WIN32

Exception::Exception(const std::string &msg, const std::string &filename, int line_number)
{
  setMessage(msg, filename, line_number);
}


Exception::Exception(Exception &&other) noexcept
  : message_(std::move(other.message_))
{}


Exception::Exception(const Exception &other) noexcept
  : message_(other.message_)
{}


Exception::~Exception() = default;


const char *Exception::what() const noexcept
{
  return message_.c_str();
}


void Exception::setMessage(const std::string &message, const std::string &filename, int line_number)
{
  std::string str;
  if (!message.empty() || !filename.empty())
  {
    std::ostringstream sstr;
    if (!filename.empty())
    {
      sstr << filename;

      if (line_number > 0)
      {
        sstr << '(' << line_number << "):";
      }
      sstr << ' ';
    }
    if (!message.empty())
    {
      sstr << message;
    }
    str = sstr.str();
  }

  message_ = std::move(str);
}
}  // namespace gputil
