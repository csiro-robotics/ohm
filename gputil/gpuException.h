// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUEXCEPTION_H
#define GPUEXCEPTION_H

#include "gpuConfig.h"
#include "gpuThrow.h"

#include <exception>

namespace gputil
{
  class gputilAPI Exception : public std::exception
  {
  public:
    Exception(const char *msg = nullptr, const char *filename = nullptr, int line_number = 0);
    Exception(Exception &&other) noexcept;

    virtual ~Exception();

    const char *what() const noexcept override;

  protected:
    void setMessage(const char *message, const char *filename = nullptr, int line_number = 0);

  private:
    char *message_;
  };
}  // namespace gputil

#endif  // GPUEXCEPTION_H
