// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUEXCEPTION_H
#define GPUEXCEPTION_H

#include "gpuConfig.h"
#include "gpuThrow.h"

namespace gputil
{
  class gputilAPI Exception
  {
  public:
    Exception(const char *msg = nullptr);
    Exception(Exception &&other) noexcept;

    virtual ~Exception();

    inline const char *message() const { return message_ ? message_ : ""; }

  protected:
    void setMessage(const char *message);

  private:
    char *message_;
  };
}

#endif // GPUEXCEPTION_H
