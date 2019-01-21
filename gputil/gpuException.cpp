// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuException.h"

using namespace gputil;

#include <cstring>

#ifdef WIN32
#define strncpy(dst, msg, len) strncpy_s(dst, len + 1, msg, len)
#endif  // WIN32

Exception::Exception(const char *msg)
  : message_(nullptr)
{
  setMessage(msg);
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


void Exception::setMessage(const char *message)
{
  delete[] message_;
  message_ = nullptr;
  if (message)
  {
    size_t msglen = strlen(message);
    message_ = new char[msglen + 1];
    strncpy(message_, message, msglen);
    message_[msglen] = '\0';
  }
}
