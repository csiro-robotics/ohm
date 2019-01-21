// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "cluKernel.h"

#include "cluProgram.h"

using namespace clu;

KernelSize KernelGrid::adjustedGlobal() const
{
  if (!isValid())
  {
    return KernelSize();
  }

  KernelSize adjusted;
  for (unsigned i = 0; i < global_size.dimensions(); ++i)
  {
    adjusted[i] = work_group_size[i] * ((global_size[i] + work_group_size[i] - 1) / work_group_size[i]);
  }

  return adjusted;
}

Kernel::Kernel()
  : optimal_work_group_size_(0)
  , local_mem_arg_count_(0)
  , local_mem_first_(false)
{}


Kernel::Kernel(cl::Program &program, const char *entry_point, std::ostream *log)
  : optimal_work_group_size_(0)
  , local_mem_arg_count_(0)
  , local_mem_first_(false)
{
  setEntry(program, entry_point, log);
}


Kernel::Kernel(cl::Kernel &cl_kernel)
  : kernel_(cl_kernel)
  , optimal_work_group_size_(0)
  , local_mem_arg_count_(0)
  , local_mem_first_(false)
{}


bool Kernel::isValid() const
{
  return kernel_() != nullptr;
}


cl_int Kernel::setEntry(cl::Program &program, const char *entry_point, std::ostream *log)
{
  std::ostream &out = (log) ? *log : std::cerr;
  cl_int clerr = CL_SUCCESS;
  kernel_ = cl::Kernel(program, entry_point, &clerr);

  if (clerr != CL_SUCCESS)
  {
    out << "Failed to resolve kernel " << entry_point << "() : " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  return clerr;
}


cl_int Kernel::addLocal(const LocalMemArgSizeFunc &arg_func)
{
  if (local_mem_arg_count_ < kMaxLocalMemArgs)
  {
    local_mem_args_[local_mem_arg_count_++] = arg_func;

    return local_mem_arg_count_;
  }

  return -1;
}


size_t Kernel::calculateOptimalWorkGroupSize()
{
  cl_int clerr = CL_SUCCESS;

  optimal_work_group_size_ = 1;

  if (local_mem_arg_count_)
  {
    clu::LocalMemCalcFunc local_mem_func = [this](size_t work_group_size) -> size_t {
      size_t mem_size = 0;
      for (int i = 0; i < local_mem_arg_count_; ++i)
      {
        mem_size += local_mem_args_[i](work_group_size);
      }

      return mem_size;
    };

    optimal_work_group_size_ = clu::maxWorkgroupSize(kernel_, local_mem_func, &clerr);
  }
  else
  {
    optimal_work_group_size_ = clu::maxWorkgroupSize(kernel_, &clerr);
  }

  return optimal_work_group_size_;
}


cl_int Kernel::setLocalMemArgs(int arg_count)
{
  int arg_index = (!local_mem_first_) ? arg_count : 0;
  cl_int clerr = CL_SUCCESS;

  for (int i = 0; i < local_mem_arg_count_; ++i)
  {
    if (local_mem_args_[i])
    {
      clerr = kernel_.setArg(arg_index++, cl::Local(local_mem_args_[i](optimal_work_group_size_)));
    }
    else
    {
      clerr = kernel_.setArg(arg_index++, cl::Local(0));
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
  cl_event *wait_on_events = nullptr;
  if (events.event_count)
  {
    wait_on_events = static_cast<cl_event *>(alloca(sizeof(cl_event) * events.event_count));
    for (unsigned i = 0; i < events.event_count; ++i)
    {
      wait_on_events[i] = events.wait_on_events[i]();
    }
  }

  cl_event local_event;

  // Invoke the kernel.
  cl_int clerr;
  clerr = clEnqueueNDRangeKernel(queue(), kernel_(),
                                 grid.global_size.dimensions(),                                      // Dimensions
                                 !grid.global_offset.isNull() ? grid.global_offset.arg() : nullptr,  // Global offset
                                 grid.adjustedGlobal().arg(),                                        // Global size
                                 grid.work_group_size.arg(),                                         // Work group size
                                 events.event_count, wait_on_events, events.completion ? &local_event : nullptr);
  if (clerr == CL_SUCCESS && events.completion)
  {
    *events.completion = local_event;
  }

  return clerr;
}
