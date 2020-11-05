//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#include "cluProgram.h"

#include "clu.h"

#include <array>
#include <fstream>
#include <ostream>
#include <sstream>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else  // WIN32
#include <unistd.h>
#endif  // WIN32

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif  // __APPLE__

namespace
{
const size_t kMaxPath = 2048u;
}  // namespace

namespace clu
{
void printBuildLog(std::ostream &out, const cl::Program &program, const cl::Device &device, int filter_status)
{
  size_t log_size = 0;
  std::string log;
  std::string device_name;
  cl_build_status status{};

  if (filter_status <= 0)
  {
    clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_STATUS, sizeof(status), &status, nullptr);
    if (status != filter_status)
    {
      return;
    }
  }

  clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
  if (log.size() < log.size() + 1)
  {
    log.resize(log_size + 1);
  }
  clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, sizeof(*log.data()) * log.size(), &log.at(0),
                        nullptr);
  device.getInfo(CL_DEVICE_NAME, &device_name);
  out << device_name << '\n';
  out << log << std::endl;
}


void printBuildLogs(std::ostream &out, const cl::Program &program, const cl::Context &context, int filter_status)
{
  size_t log_size = 0;
  std::string log;
  std::string device_name;
  std::vector<cl::Device> devices;
  cl_build_status status{};

  listDevices(devices, context);

  for (cl::Device &device : devices)
  {
    if (filter_status <= 0)
    {
      clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_STATUS, sizeof(status), &status, nullptr);
      if (status != filter_status)
      {
        continue;
      }
    }

    clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_size);
    if (log.size() < log.size() + 1)
    {
      log.resize(log_size + 1);
    }
    clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, sizeof(*log.data()) * log.size(), &log.at(0),
                          nullptr);
    device.getInfo(CL_DEVICE_NAME, &device_name);
    out << device_name << '\n';
    out << log << std::endl;
  }
}

unsigned listDevices(std::vector<cl::Device> &devices, const cl::Program &program)
{
  cl_uint device_count = 0;
  devices.clear();
  cl_int clerr = clGetProgramInfo(program(), CL_PROGRAM_NUM_DEVICES, sizeof(device_count), &device_count, nullptr);

  if (device_count == 0 || clerr != CL_SUCCESS)
  {
    return 0;
  }

  auto *device_ids = static_cast<cl_device_id *>(alloca(sizeof(cl_device_id) * device_count));
  clerr = clGetProgramInfo(program(), CL_PROGRAM_DEVICES, sizeof(cl_device_id) * device_count, device_ids, nullptr);
  if (clerr != CL_SUCCESS)
  {
    return 0;
  }

  devices.resize(device_count);
  for (cl_uint i = 0; i < device_count; ++i)
  {
    devices[i] = cl::Device(device_ids[i]);
  }

  return device_count;
}

size_t maxWorkgroupSize(const cl::Kernel &kernel, const LocalMemCalcFunc &local_mem_func, cl_int *err)
{
  cl_program program = nullptr;
  cl_int clerr = clGetKernelInfo(kernel(), CL_KERNEL_PROGRAM, sizeof(cl_program), &program, nullptr);

  if (clerr != CL_SUCCESS)
  {
    if (err)
    {
      *err = clerr;
    }
    return 0;
  }

  cl_uint device_count = 0;
  clerr = clGetProgramInfo(program, CL_PROGRAM_NUM_DEVICES, sizeof(device_count), &device_count, nullptr);

  if (device_count == 0 || clerr != CL_SUCCESS)
  {
    return 0;
  }

  auto *device_ids = static_cast<cl_device_id *>(alloca(sizeof(cl_device_id) * device_count));
  clerr = clGetProgramInfo(program, CL_PROGRAM_DEVICES, sizeof(cl_device_id) * device_count, device_ids, nullptr);
  if (clerr != CL_SUCCESS)
  {
    if (err)
    {
      *err = clerr;
    }
    return 0;
  }

  size_t max_items = 0;

  // Iterate looking for a compatible device. I've had the experience that the program is
  // associated with multiple devices, but just taking the first device results in a
  // CL_INVALID_DEVICE result from clGetKernelWorkGroupInfo(). This strategy
  // works around the issue, ensuring success.
  for (cl_uint i = 0; max_items == 0 && i < device_count; ++i)
  {
    cl_device_id device = device_ids[i];

    // size_t maxItems1 = 0;
    cl_kernel kernel_obj = kernel();
    // Interesting note: using Apple OpenCL 1.2 and the CPU driver can give what seems to be
    // odd results here for CL_KERNEL_WORK_GROUP_SIZE. Specifically I noted that it kept being
    // reported as 1. Playing with the OpenCL code, I found that it dropped from 'reasonable values'
    // (say 128) to 1 as soon as the OpenCL call included a call to barrier(), either local or global.
    // I guess that the thread synchronisation for this implementation is not very good, so it simply
    // drops the resolution back to avoid having any synchronisation code.
    clerr =
      clGetKernelWorkGroupInfo(kernel_obj, device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(max_items), &max_items, nullptr);
    if (clerr != CL_SUCCESS)
    {
      if (clerr == CL_INVALID_DEVICE)
      {
        // Try the next device.
        continue;
      }

      if (err)
      {
        *err = clerr;
      }
      return 0;
    }

    if (local_mem_func)
    {
      const size_t upper_limit = max_items;
      size_t last_limit = 0;
      size_t local_mem_used = 0;
      cl_ulong kernel_local_memory = 0;
      cl_ulong kernel_private_memory = 0;
      cl_ulong available_local_memory = 0;

      // Get the base memory used by the kernel.
      clerr = clGetKernelWorkGroupInfo(kernel_obj, device, CL_KERNEL_LOCAL_MEM_SIZE, sizeof(kernel_local_memory),
                                       &kernel_local_memory, nullptr);
      if (clerr != CL_SUCCESS)
      {
        if (err)
        {
          *err = clerr;
        }
        return 0;
      }
      clerr = clGetKernelWorkGroupInfo(kernel_obj, device, CL_KERNEL_PRIVATE_MEM_SIZE, sizeof(kernel_private_memory),
                                       &kernel_private_memory, nullptr);
      if (clerr != CL_SUCCESS)
      {
        if (err)
        {
          *err = clerr;
        }
        return 0;
      }

      // Get the max local memory.
      clerr = clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(available_local_memory), &available_local_memory,
                              nullptr);
      if (clerr != CL_SUCCESS)
      {
        if (err)
        {
          *err = clerr;
        }
        return 0;
      }

      if (available_local_memory < kernel_local_memory + kernel_private_memory)
      {
        // Failed.
        return 0;
      }

      // Subtract the kernel overhead.
      available_local_memory -= kernel_local_memory;
      available_local_memory -= kernel_private_memory;

      // Ensure some overhead.
      if (kernel_local_memory == 0 && kernel_private_memory == 0)
      {
        available_local_memory -= 1;
      }

      // Loop to find a reasonable value.
      const int loop_limit = 8;
      int loops = 0;

      // Prime the loop.
      max_items = max_items << 1u;
      do
      {
        last_limit = max_items;
        max_items = max_items >> 1u;
        local_mem_used = local_mem_func(max_items);
      } while (local_mem_used > available_local_memory && loops++ < loop_limit);

      if (local_mem_used > available_local_memory)
      {
        // Failed.
        return 0;
      }

      if (max_items < upper_limit)
      {
        // We have a viable value. Now try increase it to increase occupancy.
        loops = 0;
        size_t test_items = max_items;
        size_t diff = 0;
        do
        {
          diff = last_limit - test_items;
          test_items += diff / 2;
          local_mem_used = local_mem_func(test_items);
          if (local_mem_used <= available_local_memory)
          {
            max_items = test_items;
          }
          last_limit = test_items;
        } while (diff > 0 && loops++ < loop_limit);
      }
    }
  }

  // clGetDeviceInfo(device_, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(maxItems1), &maxItems1, nullptr);
  return max_items;
}


char *upDir(char *path)
{
  const char path_sep = char(pathSeparator());
  size_t last_dir_pos = std::char_traits<char>::length(path);

  while (last_dir_pos > 0)
  {
    if (path[last_dir_pos] == path_sep)
    {
      break;
    }

    --last_dir_pos;
  }

  path[last_dir_pos] = '\0';
  return path;
}


bool executablePath(char *buffer, size_t buffer_size)
{
  if (buffer && buffer_size)
  {
#if defined(WIN32)
    GetModuleFileNameA(nullptr, buffer, static_cast<DWORD>(buffer_size));
    buffer[buffer_size - 1] = '\0';
    return true;
#elif defined(__APPLE__)
    uint32_t size = uint32_t(buffer_size);
    if (_NSGetExecutablePath(buffer, &size) == 0)
    {
      buffer[buffer_size - 1] = '\0';
      return true;
    }
#else
    int read = ::readlink("/proc/self/exe", buffer, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    if (read != -1)
    {
      return true;
    }
#endif  // WIN32
  }
  return false;
}


size_t applicationDirectory(char *buffer, size_t buffer_size)
{
  std::array<char, kMaxPath> path{};
  size_t path_length = path.size();
  if (executablePath(path.data(), path_length))
  {
    path[path_length - 1] = '\0';  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    upDir(path.data());

    path_length = std::char_traits<char>::length(path.data());
    if (buffer && buffer_size)
    {
      if (buffer_size <= path_length)
      {
        // Truncate
        path[buffer_size - 1] = '\0';  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
      }

      std::char_traits<char>::copy(buffer, path.data(), path_length + 1);
    }

    return path_length;
  }

  return 0;
}


char *currentWorkingDirectory(char *cwd, size_t buffer_length)
{
#ifdef WIN32
  GetCurrentDirectoryA(static_cast<DWORD>(buffer_length), cwd);
#else   // WIN32
  // const char *ignore =
  cwd = getcwd(cwd, buffer_length);
#endif  // WIN32
  // Guarantee null termination.
  cwd[buffer_length - 1] = '\0';
  return cwd;
}


int pathSeparator()
{
#ifdef WIN32
  return '\\';
#else   // WIN32
  return '/';
#endif  // WIN32
}

char *findProgramPath(char *file_name, size_t buffer_length, const char *search_paths)
{
  // Resolve the current working directory first.
  std::array<char, kMaxPath> cwd{};
  currentWorkingDirectory(cwd.data(), cwd.size());
  cwd[cwd.size() - 1] = '\0';
  std::ostringstream str;
  str << cwd.data() << char(pathSeparator()) << file_name;

  std::ifstream infile;
  infile.open(str.str());
  if (infile.is_open())
  {
    infile.close();
    // File is in current working directory.
#ifdef _MSC_VER
    strcpy_s(file_name, buffer_length, str.str().c_str());
#else   // _MSC_VER
    strncpy(file_name, str.str().c_str(), buffer_length);
#endif  // _MSC_VER
    return file_name;
  }

  // Try the application directory.
  applicationDirectory(cwd.data(), cwd.size());
  str.str(std::string());
  str << cwd.data() << char(pathSeparator()) << file_name;
  infile.open(str.str());
  if (infile.is_open())
  {
    infile.close();
    // File is in application directory.
#ifdef _MSC_VER
    strncpy_s(file_name, buffer_length, str.str().c_str(), buffer_length);
#else   // _MSC_VER
    strncpy(file_name, str.str().c_str(), buffer_length);
#endif  // _MSC_VER
    file_name[buffer_length - 1] = '\0';
    return file_name;
  }

  // Try search paths.
  if (search_paths && search_paths[0])
  {
    const char *path = search_paths;
    const char *ch = path;
    size_t path_len = 0;
    do
    {
      if (*ch == ',' || *ch == '\0')
      {
        path_len = size_t(ch - path);
        if (path_len >= cwd.size())
        {
          path_len = cwd.size() - 1;
        }

        if (path_len)
        {
#ifdef _MSC_VER
          strncpy_s(cwd.data(), path, path_len);
#else   // _MSC_VER
          strncpy(cwd.data(), path, path_len);
#endif  // _MSC_VER

          cwd[path_len] = '\0';  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          // Try the path.
          str.str(std::string());
          str << cwd.data();
          if (cwd[path_len - 1] != pathSeparator())  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          {
            str << char(pathSeparator());
          }
          str << file_name;
          infile.open(str.str());
          if (infile.is_open())
          {
            // Found it.
            infile.close();
            // File is on a search path.
#ifdef _MSC_VER
            strncpy_s(file_name, buffer_length, str.str().c_str(), buffer_length);
#else   // _MSC_VER
            strncpy(file_name, str.str().c_str(), buffer_length);
#endif  // _MSC_VER
            file_name[buffer_length - 1] = '\0';
            return file_name;
          }

          // Adjust start of next path pointer.
          path = ch;
          do
          {
            ++path;
          } while (*path == ',');
        }
      }
      ++ch;
    } while (ch[-1]);
  }
  // Not found.
  return nullptr;
}


char *findProgramDir(char *file_name, size_t buffer_length, const char *search_paths)
{
  // Resolve the current working directory first.
  std::array<char, kMaxPath> cwd{};
  currentWorkingDirectory(cwd.data(), cwd.size());
  std::ostringstream str;
  str << cwd.data() << char(pathSeparator()) << file_name;
  // std::cout << "First try: " << str.str() << std::endl;

  std::ifstream infile;
  infile.open(str.str());
  if (infile.is_open())
  {
    infile.close();
    // File is in current working directory.
#ifdef _MSC_VER
    strcpy_s(file_name, buffer_length, cwd.data());
#else   // _MSC_VER
    strncpy(file_name, cwd.data(), buffer_length);
#endif  // _MSC_VER
    return file_name;
  }

  // Try the application directory.
  applicationDirectory(cwd.data(), cwd.size());
  str.str(std::string());
  str << cwd.data() << char(pathSeparator()) << file_name;
  // std::cout << "try: " << str.str() << std::endl;
  infile.open(str.str());
  if (infile.is_open())
  {
    infile.close();
    // File is in application directory.
#ifdef _MSC_VER
    strncpy_s(file_name, buffer_length, cwd.data(), buffer_length);
#else   // _MSC_VER
    strncpy(file_name, cwd.data(), buffer_length);
#endif  // _MSC_VER
    file_name[buffer_length - 1] = '\0';
    return file_name;
  }

  // Try search paths.
  if (search_paths && search_paths[0])
  {
    const char *path = search_paths;
    const char *ch = path;
    size_t path_len = 0;
    do
    {
      if (*ch == ',' || *ch == '\0')
      {
        path_len = size_t(ch - path);
        if (path_len >= cwd.size())
        {
          path_len = cwd.size() - 1;
        }

        if (path_len)
        {
#ifdef _MSC_VER
          strncpy_s(cwd.data(), path, path_len);
#else   // _MSC_VER
          strncpy(cwd.data(), path, path_len);
#endif  // _MSC_VER

          cwd[path_len] = '\0';  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          // Try the path.
          str.str(std::string());
          str << cwd.data();
          if (cwd[path_len - 1] != pathSeparator())  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
          {
            str << char(pathSeparator());
          }
          str << file_name;
          infile.open(str.str());
          if (infile.is_open())
          {
            // Found it.
            infile.close();
            // File is on a search path.
#ifdef _MSC_VER
            strncpy_s(file_name, buffer_length, cwd.data(), buffer_length);
#else   // _MSC_VER
            strncpy(file_name, cwd.data(), buffer_length);
#endif  // _MSC_VER
            file_name[buffer_length - 1] = '\0';
            return file_name;
          }

          // Adjust start of next path pointer.
          path = ch;
          do
          {
            ++path;
          } while (*path == ',');
        }
      }
      ++ch;
    } while (ch[-1]);
  }

  // Not found.
  return nullptr;
}

cl_int buildProgramFromFile(cl::Program &program, cl::Context &ocl, std::string &source_file_name, std::ostream &log,
                            const char *args, const char *debug_option, const char *source_file_opt,
                            const char *search_paths)
{
  if (program())
  {
    // Already initialised.
    return CL_SUCCESS;
  }

  // Compile and initialise.
  std::array<char, kMaxPath> source_dir{};  // MAX_PATH length.
  std::ostringstream source_file;
#ifdef _MSC_VER
  strcpy_s(source_dir.data(), source_file_name.c_str());
#else   // !_MSC_VER
  strncpy(source_dir.data(), source_file_name.c_str(), kMaxPath);
#endif  // _MSC_VER
  if (!clu::findProgramDir(source_dir.data(), source_dir.size(), search_paths))
  {
    // Not found.
    log << "Failed to find CL source file: " << source_file_name << std::endl;
    return 1;
  }
  // log << "Found in " << source_dir.data() << std::endl;
  source_file << source_dir.data() << char(pathSeparator()) << source_file_name;
  // log << "load from " << sourceFile.str() << std::endl;
  std::ifstream file(source_file.str());
  if (!file.is_open())
  {
    // Not found.
    log << "Failed to open CL source file: " << source_file_name << std::endl;
    return 1;
  }
  source_file_name = source_file.str();

  // Setup additional build options such as include dirs.
  std::ostringstream build_opt;

  build_opt << "-I " << source_dir.data();

  if (debug_option && debug_option[0])
  {
    build_opt << ' ' << debug_option;
  }
  if (args && args[0])
  {
    build_opt << ' ' << args;
  }

  if (source_file_opt && source_file_opt[0])
  {
    build_opt << ' ' << source_file_opt << "\"" << source_file_name << "\"";
  }

  std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
  std::string build_opt_str = build_opt.str();
  std::string source_file_str = source_file.str();
  // No need to pass debugOpt. Already in buildOptStr.
  return buildProgramFromString(program, ocl, prog.c_str(), prog.length(), log, source_file_str.c_str(),
                                build_opt_str.c_str(), nullptr);
}

cl_int buildProgramFromString(cl::Program &program, cl::Context &ocl, const char *source_code, size_t source_length,
                              std::ostream &log, const char *reference_name, const char *build_args,
                              const char *debug_option)
{
  if (source_length == 0)
  {
    source_length = strlen(source_code);
  }

  cl::Program::Sources source(1, cl::string(source_code, source_length));
  cl::Program local_program(ocl, source);

  // Setup additional build options such as include dirs.
  std::ostringstream build_opt;
  bool first_opt = true;

  auto prefix_opt = [&first_opt](std::ostream &out) -> std::ostream & {
    if (!first_opt)
    {
      out << ' ';
    }
    first_opt = false;
    return out;
  };

  if (build_args && build_args[0])
  {
    prefix_opt(build_opt) << build_args;
  }

  if (debug_option && debug_option[0])
  {
    prefix_opt(build_opt) << debug_option;
  }

  cl_int clerr = local_program.build(build_opt.str().c_str());

  if (clerr == CL_SUCCESS)
  {
    program = local_program;
  }
  else
  {
    log << "Failed to build " << reference_name << ": " << clu::errorCodeString(clerr) << '\n';
    clu::printBuildLogs(log, local_program, ocl, CL_BUILD_ERROR);
  }

  return clerr;
}
}  // namespace clu
