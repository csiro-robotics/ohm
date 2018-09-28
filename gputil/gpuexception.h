// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUEXCEPTION_H_
#define GPUEXCEPTION_H_

#include "gpuconfig.h"
#include "gputhrow.h"

namespace gputil
{
  class gputilAPI Exception
  {
  public:
    Exception(const char *msg = nullptr);
    Exception(Exception &&other);

    virtual ~Exception();

    inline const char *message() const { return _message ? _message : ""; }

  protected:
    void setMessage(const char *message);

  private:
    char *_message;
  };
}

#endif // GPUEXCEPTION_H_
