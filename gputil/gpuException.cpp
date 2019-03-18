// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuException.h"
#include <cstdio>
#include <sstream>

using namespace gputil;

#include <cstring>

#ifdef WIN32
#define strncpy(dst, msg, len) strncpy_s(dst, len + 1, msg, len)
#endif  // WIN32

Exception::Exception(const char *msg, const char *filename, int line_number)
  : message_(nullptr)
{
  setMessage(msg, filename, line_number);
}


Exception::Exception(Exception &&other) noexcept
  : message_(other.message_)
{
  other.message_ = nullptr;
}


Exception::~Exception()
{
  delete[] message_;
}


const char *Exception::what() const noexcept
{
  return message_ ? message_ : "";
}


void Exception::setMessage(const char *message, const char *filename, int line_number)
{
  std::string str;
  if (message || filename)
  {
    std::ostringstream sstr;
    if (filename)
    {
      sstr << filename;
      if (line_number > 0)
      {
        sstr << '(' << line_number << "):";
      }
      sstr << ' ';
    }
    if (message)
    {
      sstr << message;
    }
    str = sstr.str();
  }

  delete[] message_;
  message_ = nullptr;
  if (str.length())
  {
    size_t msglen = str.length();
    message_ = new char[msglen + 1];
    strncpy(message_, str.c_str(), msglen);
    message_[msglen] = '\0';
  }
}
