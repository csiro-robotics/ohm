// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "clukernel.h"

#include "cluprogram.h"

#include <cstring>

using namespace clu;

KernelSize KernelGrid::adjustedGlobal() const
{
  if (!isValid())
  {
    return KernelSize();
  }

  KernelSize adjusted;
  for (unsigned i = 0; i < globalSize.dimensions(); ++i)
  {
    adjusted[i] = workGroupSize[i] * ((globalSize[i] + workGroupSize[i] - 1) / workGroupSize[i]);
  }

  return adjusted;
}

Kernel::Kernel()
  : _optimalWorkGroupSize(0)
  , _localMemArgCount(0)
  , _localMemFirst(false)
{
}


Kernel::Kernel(cl::Program &program, const char *entryPoint, std::ostream *log)
  : _optimalWorkGroupSize(0)
  , _localMemArgCount(0)
  , _localMemFirst(false)
{
  setEntry(program, entryPoint, log);
}


Kernel::Kernel(cl::Kernel &clKernel)
  : _kernel(clKernel)
  , _optimalWorkGroupSize(0)
  , _localMemArgCount(0)
  , _localMemFirst(false)
{

}


bool Kernel::isValid() const
{
  return _kernel() != nullptr;
}


cl_int Kernel::setEntry(cl::Program &program, const char *entryPoint, std::ostream *log)
{
  std::ostream &out = (log) ? *log : std::cerr;
  cl_int clerr = CL_SUCCESS;
  _kernel = cl::Kernel(program, entryPoint, &clerr);

  if (clerr != CL_SUCCESS)
  {
    out << "Failed to resolve kernel " << entryPoint << "() : " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  return clerr;
}


cl_int Kernel::addLocal(const LocalMemArgSizeFunc &argFunc)
{
  if (_localMemArgCount < MAX_LOCAL_MEM_ARGS)
  {
    _localMemArgs[_localMemArgCount++] = argFunc;

    return _localMemArgCount;
  }

  return -1;
}


size_t Kernel::calculateOptimalWorkGroupSize()
{
  cl_int clerr = CL_SUCCESS;

  _optimalWorkGroupSize = 1;

  if (_localMemArgCount)
  {
    clu::LocalMemCalcFunc localMemFunc = [this] (size_t workGroupSize) -> size_t
    {
      size_t memSize = 0;
      for (int i = 0; i < _localMemArgCount; ++i)
      {
        memSize += _localMemArgs[i](workGroupSize);
      }

      return memSize;
    };

    _optimalWorkGroupSize = clu::maxWorkgroupSize(_kernel, localMemFunc, &clerr);
  }
  else
  {
    _optimalWorkGroupSize = clu::maxWorkgroupSize(_kernel, &clerr);
  }

  return _optimalWorkGroupSize;
}


cl_int Kernel::setLocalMemArgs(int argCount)
{
  int argIndex = (!_localMemFirst) ? argCount : 0;
  cl_int clerr = CL_SUCCESS;

  for (int i = 0; i < _localMemArgCount; ++i)
  {
    if (_localMemArgs[i])
    {
      clerr = _kernel.setArg(argIndex++, cl::Local(_localMemArgs[i](_optimalWorkGroupSize)));
    }
    else
    {
      clerr = _kernel.setArg(argIndex++, cl::Local(0));
    }

    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }
  }

  return clerr;
}


cl_int Kernel::invoke(cl::CommandQueue &queue, const KernelGrid &grid, const EventList &events)
{
  // Copy wait on event into a local stack list.
  cl_event *waitOnEvents = nullptr;
  if (events.eventCount)
  {
    waitOnEvents = (cl_event *)alloca(sizeof(cl_event) * events.eventCount);
    for (unsigned i = 0; i < events.eventCount; ++i)
    {
      waitOnEvents[i] = events.waitOnEvents[i]();
    }
  }

  cl_event localEvent;

  // Invoke the kernel.
  cl_int clerr;
  clerr = clEnqueueNDRangeKernel(queue(), _kernel(),
                                grid.globalSize.dimensions(), // Dimensions
                                !grid.globalOffset.isNull() ? grid.globalOffset.arg() : nullptr,  // Global offset
                                grid.adjustedGlobal().arg(),    // Global size
                                grid.workGroupSize.arg(), // Work group size
                                events.eventCount,
                                waitOnEvents,
                                events.completion ? &localEvent : nullptr
                                );
  if (clerr == CL_SUCCESS && events.completion)
  {
    *events.completion = localEvent;
  }

  return clerr;
}
