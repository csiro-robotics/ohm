// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpuexception.h"

using namespace gputil;

#include <cstring>

#ifdef WIN32
#define strncpy(dst, msg, len) strncpy_s(dst, len + 1, msg, len)
#endif // WIN32

Exception::Exception(const char *msg)
  : _message(nullptr)
{
  setMessage(msg);
}


Exception::Exception(Exception &&other)
  : _message(other._message)
{
  other._message = nullptr;
}


Exception::~Exception()
{
  delete[] _message;
}


void Exception::setMessage(const char *message)
{
  delete[] _message;
  _message = nullptr;
  if (message)
  {
    size_t msglen = strlen(message);
    _message = new char[msglen + 1];
    strncpy(_message, message, msglen);
    _message[msglen] = '\0';
  }
}
