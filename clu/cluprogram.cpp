//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#include "cluprogram.h"

#include "clu.h"

#include <ostream>
#include <sstream>
#include <fstream>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else  // WIN32
#include <unistd.h>
#endif // WIN32

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif // __APPLE__

namespace clu
{
  void printBuildLog(std::ostream &out, const cl::Program &program, const cl::Device &device, int filterStatus)
  {
    size_t logSize = 0;
    std::string log, deviceName;
    cl_build_status status;

    if (filterStatus <= 0)
    {
      clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_STATUS, sizeof(status), &status, nullptr);
      if (status != filterStatus)
      {
        return;
      }
    }

    clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
    if (log.size() < log.size() + 1)
    {
      log.resize(logSize + 1);
    }
    clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, sizeof(*log.data()) * log.size(), &log.at(0), nullptr);
    device.getInfo(CL_DEVICE_NAME, &deviceName);
    out << deviceName << '\n';
    out << log << std::endl;
  }


  void printBuildLogs(std::ostream &out, const cl::Program &program, const cl::Context &context, int filterStatus)
  {
    size_t logSize = 0;
    std::string log, deviceName;
    std::vector<cl::Device> devices;
    cl_build_status status;

    listDevices(devices, context);

    for (cl::Device &device : devices)
    {
      if (filterStatus <= 0)
      {
        clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_STATUS, sizeof(status), &status, nullptr);
        if (status != filterStatus)
        {
          continue;
        }
      }

      clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &logSize);
      if (log.size() < log.size() + 1)
      {
        log.resize(logSize + 1);
      }
      clGetProgramBuildInfo(program(), device(), CL_PROGRAM_BUILD_LOG, sizeof(*log.data()) * log.size(), &log.at(0), nullptr);
      device.getInfo(CL_DEVICE_NAME, &deviceName);
      out << deviceName << '\n';
      out << log << std::endl;
    }
  }

  unsigned listDevices(std::vector<cl::Device> &devices, const cl::Program &program)
  {
    cl_int clerr = 0;
    cl_int deviceCount = 0;
    devices.clear();
    clerr = clGetProgramInfo(program(), CL_PROGRAM_NUM_DEVICES, sizeof(deviceCount), &deviceCount, nullptr);

    if (deviceCount == 0 || clerr != CL_SUCCESS)
    {
      return 0;
    }

    cl_device_id *deviceIds = (cl_device_id *)alloca(sizeof(cl_device_id) * deviceCount);
    clerr = clGetProgramInfo(program(), CL_PROGRAM_DEVICES, sizeof(cl_device_id) * deviceCount, deviceIds, nullptr);
    if (clerr != CL_SUCCESS)
    {
      return 0;
    }

    devices.resize(deviceCount);
    for (cl_int i = 0; i < deviceCount; ++i)
    {
      devices[i] = cl::Device(deviceIds[i]);
    }

    return deviceCount;
  }

  size_t maxWorkgroupSize(const cl::Kernel &kernel,
                          const LocalMemCalcFunc &localMemFunc,
                          cl_int *err)
  {
    cl_int clerr = CL_SUCCESS;
    cl_program program = nullptr;
    clerr = clGetKernelInfo(kernel(), CL_KERNEL_PROGRAM, sizeof(cl_program), &program, nullptr);

    if (clerr != CL_SUCCESS)
    {
      if (err)
      {
        *err = clerr;
      }
      return 0;
    }

    cl_uint deviceCount = 0;
    clerr = clGetProgramInfo(program, CL_PROGRAM_NUM_DEVICES, sizeof(deviceCount), &deviceCount, nullptr);

    if (deviceCount == 0 || clerr != CL_SUCCESS)
    {
      return 0;
    }

    cl_device_id *deviceIds = (cl_device_id *)alloca(sizeof(cl_device_id) * deviceCount);
    clerr = clGetProgramInfo(program, CL_PROGRAM_DEVICES, sizeof(cl_device_id) * deviceCount, deviceIds, nullptr);
    if (clerr != CL_SUCCESS)
    {
      if (err)
      {
        *err = clerr;
      }
      return 0;
    }

    size_t maxItems = 0;

    // Iterate looking for a compatible device. I've had the experience that the program is
    // associated with multiple devices, but just taking the first device results in a
    // CL_INVALID_DEVICE result from clGetKernelWorkGroupInfo(). This strategy
    // works around the issue, ensuring success.
    for (cl_uint i = 0; maxItems == 0 && i < deviceCount; ++i)
    {
      cl_device_id device = deviceIds[i];

      //size_t maxItems1 = 0;
      cl_kernel kernel_ = kernel();
      // Interesting note: using Apple OpenCL 1.2 and the CPU driver can give what seems to be
      // odd results here for CL_KERNEL_WORK_GROUP_SIZE. Specifically I noted that it kept being
      // reported as 1. Playing with the OpenCL code, I found that it dropped from 'reasonable values'
      // (say 128) to 1 as soon as the OpenCL call included a call to barrier(), either local or global.
      // I guess that the thread synchronisation for this implementation is not very good, so it simply
      // drops the resolution back to avoid having any synchronisation codef.
      clerr = clGetKernelWorkGroupInfo(kernel_, device, CL_KERNEL_WORK_GROUP_SIZE,
                                       sizeof(maxItems), &maxItems, nullptr);
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

      if (localMemFunc)
      {
        const size_t upperLimit = maxItems;
        size_t lastLimit;
        size_t localMemUsed = 0;
        cl_ulong kernelLocalMemory = 0;
        cl_ulong kernelPrivateMemory = 0;
        cl_ulong availableLocalMemory = 0;

        // Get the base memory used by the kernel.
        clerr = clGetKernelWorkGroupInfo(kernel_, device, CL_KERNEL_LOCAL_MEM_SIZE,
                                         sizeof(kernelLocalMemory), &kernelLocalMemory, nullptr);
        if (clerr != CL_SUCCESS)
        {
          if (err)
          {
            *err = clerr;
          }
          return 0;
        }
        clerr = clGetKernelWorkGroupInfo(kernel_, device, CL_KERNEL_PRIVATE_MEM_SIZE,
                                         sizeof(kernelPrivateMemory), &kernelPrivateMemory, nullptr);
        if (clerr != CL_SUCCESS)
        {
          if (err)
          {
            *err = clerr;
          }
          return 0;
        }

        // Get the max local memory.
        clerr = clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(availableLocalMemory), &availableLocalMemory, nullptr);
        if (clerr != CL_SUCCESS)
        {
          if (err)
          {
            *err = clerr;
          }
          return 0;
        }

        if (availableLocalMemory < kernelLocalMemory + kernelPrivateMemory)
        {
          // Failed.
          return 0;
        }

        // Subtract the kernel overhead.
        availableLocalMemory -= kernelLocalMemory;
        availableLocalMemory -= kernelPrivateMemory;

        // Ensure some overhead.
        if (kernelLocalMemory == 0 && kernelPrivateMemory == 0)
        {
          availableLocalMemory -= 1;
        }

        // Loop to find a reasonable value.
        const int loopLimit = 8;
        int loops = 0;

        // Prime the loop.
        maxItems = maxItems << 1;
        do
        {
          lastLimit = maxItems;
          maxItems = maxItems >> 1;
          localMemUsed = localMemFunc(maxItems);
        }
        while (localMemUsed > availableLocalMemory && loops++ < loopLimit);

        if (localMemUsed > availableLocalMemory)
        {
          // Failed.
          return 0;
        }

        if (maxItems < upperLimit)
        {
          // We have a viable value. Now try increase it to increase occupancy.
          loops = 0;
          size_t testItems = maxItems;
          size_t diff;
          do
          {
            diff = lastLimit - testItems;
            testItems += diff / 2;
            localMemUsed = localMemFunc(testItems);
            if (localMemUsed <= availableLocalMemory)
            {
              maxItems = testItems;
            }
            lastLimit = testItems;
          }
          while (diff > 0 && loops++ < loopLimit);
        }
      }
    }

    //clGetDeviceInfo(device_, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(maxItems1), &maxItems1, nullptr);
    return maxItems;
  }


  char *upDir(char *path)
  {
    size_t lastDirPos;
    const char pathSep = pathSeparator();

    lastDirPos = std::char_traits<char>::length(path);

    while (lastDirPos > 0)
    {
      if (path[lastDirPos] == pathSep)
      {
        break;
      }

      --lastDirPos;
    }

    path[lastDirPos] = '\0';
    return path;
  }


  bool executablePath(char *buffer, size_t bufferSize)
  {
    if (buffer && bufferSize)
    {
#if defined(WIN32)
      GetModuleFileNameA(0, buffer, (DWORD)bufferSize);
      buffer[bufferSize - 1] = '\0';
      return true;
#elif defined(__APPLE__)
      uint32_t size = bufferSize;
      if (_NSGetExecutablePath(buffer, &size) == 0)
      {
        buffer[bufferSize - 1] = '\0';
        return true;
      }
#else
      int read = ::readlink("/proc/self/exe", buffer, bufferSize - 1);
      buffer[bufferSize - 1] = '\0';
      if (read != -1)
      {
        return true;
      }
#endif // WIN32
    }
    return false;
  }


  size_t applicationDirectory(char *buffer, size_t bufferSize)
  {
    char path[1024u];
    size_t pathLength = sizeof(path) / sizeof(char);
    if (executablePath(path, pathLength))
    {
      path[pathLength - 1] = '\0';
      upDir(path);

      pathLength = std::char_traits<char>::length(path);
      if (buffer && bufferSize)
      {
        if (bufferSize <= pathLength)
        {
          // Truncate
          path[bufferSize - 1] = '\0';
        }

        std::char_traits<char>::copy(buffer, path, pathLength + 1);
      }

      return pathLength;
    }

    return 0;
  }


  char *currentWorkingDirectory(char *path, size_t bufferLength)
  {
#ifdef WIN32
    GetCurrentDirectoryA((DWORD)bufferLength, path);
#else  // WIN32
    const char *ignore = getcwd(path, bufferLength);
#endif // WIN32
    // Guarantee null termination.
    path[bufferLength - 1] = '\0';
    return path;
  }


  int pathSeparator()
  {
#ifdef WIN32
    return '\\';
#else  // WIN32
    return '/';
#endif // WIN32
  }


  char *findProgramPath(char *fileName, size_t bufferLength, const char *searchPaths)
  {
    // Resolve the current working directory first.
    // Size determined by Windows MAX_PATH_LEN
    const size_t CWD_MAX = 260;
    char cwd[CWD_MAX];
    currentWorkingDirectory(cwd, CWD_MAX);
    std::ostringstream str;
    str << cwd << (char)pathSeparator() << fileName;

    std::ifstream infile;
    infile.open(str.str());
    if (infile.is_open())
    {
      infile.close();
      // File is in current working directory.
#ifdef _MSC_VER
      strcpy_s(fileName, bufferLength, str.str().c_str());
#else // _MSC_VER
      strcpy(fileName, str.str().c_str());
#endif // _MSC_VER
      return fileName;
    }

    // Try the application directory.
    applicationDirectory(cwd, CWD_MAX);
    str.str(std::string());
    str << cwd << (char)pathSeparator() << fileName;
    infile.open(str.str());
    if (infile.is_open())
    {
      infile.close();
      // File is in application directory.
#ifdef _MSC_VER
      strncpy_s(fileName, bufferLength, str.str().c_str(), bufferLength);
#else // _MSC_VER
      strncpy(fileName, str.str().c_str(), bufferLength);
#endif // _MSC_VER
      fileName[bufferLength - 1] = '\0';
      return fileName;
    }

    // Try search paths.
    if (searchPaths && searchPaths[0])
    {
      const char *path = searchPaths;
      const char *ch = path;
      size_t pathLen;
      do
      {
        if (*ch == ',' || *ch == '\0')
        {
          pathLen = ch - path;
          if (pathLen >= CWD_MAX)
          {
            pathLen = CWD_MAX - 1;
          }

          if (pathLen)
          {
#ifdef _MSC_VER
            strncpy_s(cwd, path, pathLen);
#else // _MSC_VER
            strncpy(cwd, path, pathLen);
#endif // _MSC_VER

            cwd[pathLen] = '\0';
            // Try the path.
            str.str(std::string());
            str << cwd;
            if (cwd[pathLen - 1] != pathSeparator())
            {
              str << (char)pathSeparator();
            }
            str << fileName;
            infile.open(str.str());
            if (infile.is_open())
            {
              // Found it.
              infile.close();
              // File is on a search path.
#ifdef _MSC_VER
              strncpy_s(fileName, bufferLength, str.str().c_str(), bufferLength);
#else // _MSC_VER
              strncpy(fileName, str.str().c_str(), bufferLength);
#endif // _MSC_VER
              fileName[bufferLength - 1] = '\0';
              return fileName;
            }

            // Adjust start of next path pointer.
            path = ch;
            do
            {
              ++path;
            }
            while (*path && *path == ',');
          }
        }
        ++ch;
      }
      while (ch[-1]);
    }
    // Not found.
    return nullptr;
  }


  char *findProgramDir(char *fileName, size_t bufferLength, const char *searchPaths)
  {
    // Resolve the current working directory first.
    // Size determined by Windows MAX_PATH_LEN
    const size_t CWD_MAX = 260;
    char cwd[CWD_MAX];
    currentWorkingDirectory(cwd, CWD_MAX);
    std::ostringstream str;
    str << cwd << (char)pathSeparator() << fileName;
    // std::cout << "First try: " << str.str() << std::endl;

    std::ifstream infile;
    infile.open(str.str());
    if (infile.is_open())
    {
      infile.close();
      // File is in current working directory.
#ifdef _MSC_VER
      strcpy_s(fileName, bufferLength, cwd);
#else // _MSC_VER
      strcpy(fileName, cwd);
#endif // _MSC_VER
      return fileName;
    }

    // Try the application directory.
    applicationDirectory(cwd, CWD_MAX);
    str.str(std::string());
    str << cwd << (char)pathSeparator() << fileName;
    // std::cout << "try: " << str.str() << std::endl;
    infile.open(str.str());
    if (infile.is_open())
    {
      infile.close();
      // File is in application directory.
#ifdef _MSC_VER
      strncpy_s(fileName, bufferLength, cwd, bufferLength);
#else // _MSC_VER
      strncpy(fileName, cwd, bufferLength);
#endif // _MSC_VER
      fileName[bufferLength - 1] = '\0';
      return fileName;
    }

    // Try search paths.
    if (searchPaths && searchPaths[0])
    {
      const char *path = searchPaths;
      const char *ch = path;
      size_t pathLen;
      do
      {
        if (*ch == ',' || *ch == '\0')
        {
          pathLen = ch - path;
          if (pathLen >= CWD_MAX)
          {
            pathLen = CWD_MAX - 1;
          }

          if (pathLen)
          {
#ifdef _MSC_VER
            strncpy_s(cwd, path, pathLen);
#else // _MSC_VER
            strncpy(cwd, path, pathLen);
#endif // _MSC_VER

            cwd[pathLen] = '\0';
            // Try the path.
            str.str(std::string());
            str << cwd;
            if (cwd[pathLen - 1] != pathSeparator())
            {
              str << (char)pathSeparator();
            }
            str << fileName;
            infile.open(str.str());
            if (infile.is_open())
            {
              // Found it.
              infile.close();
              // File is on a search path.
#ifdef _MSC_VER
              strncpy_s(fileName, bufferLength, cwd, bufferLength);
#else // _MSC_VER
              strncpy(fileName, cwd, bufferLength);
#endif // _MSC_VER
              fileName[bufferLength - 1] = '\0';
              return fileName;
            }

            // Adjust start of next path pointer.
            path = ch;
            do
            {
              ++path;
            }
            while (*path && *path == ',');
          }
        }
        ++ch;
      }
      while (ch[-1]);
    }

    // Not found.
    return nullptr;
  }

  cl_int buildProgramFromFile(cl::Program &program, cl::Context &ocl,
                              std::string &sourceFileName,
                              std::ostream &log,
                              const std::vector<std::string> *args,
                              const char *debugOption,
                              const char *sourceFileOpt,
                              const char *searchPaths)
  {
    if (program())
    {
      // Already initialised.
      return CL_SUCCESS;
    }

    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    char sourceDir[260]; // MAX_PATH length.
    std::ostringstream sourceFile;
#ifdef _MSC_VER
    strcpy_s(sourceDir, sourceFileName.c_str());
#else  // !_MSC_VER
    strcpy(sourceDir, sourceFileName.c_str());
#endif // _MSC_VER
    if (!clu::findProgramDir(sourceDir, sizeof(sourceDir), searchPaths))
    {
      // Not found.
      log << "Failed to find CL source file: " << sourceFileName << std::endl;
      return 1;
    }
    // log << "Found in " << sourceDir << std::endl;
    sourceFile << sourceDir << (char)pathSeparator() << sourceFileName;
    // log << "load from " << sourceFile.str() << std::endl;
    std::ifstream file(sourceFile.str());
    if (!file.is_open())
    {
      // Not found.
      log << "Failed to open CL source file: " << sourceFileName << std::endl;
      return 1;
    }
    sourceFileName = sourceFile.str();

    // Setup additional build options such as include dirs.
    std::ostringstream buildOpt;

    buildOpt << "-I " << sourceDir;

    if (debugOption && debugOption[0])
    {
      buildOpt << ' ' << debugOption;
    }
    if (args)
    {
      for (size_t i = 0; i < args->size(); ++i)
      {
        buildOpt << ' ' << (*args)[i];
      }
    }
    if (sourceFileOpt)
    {
      buildOpt << ' ' << sourceFileOpt << "\"" << sourceFileName << "\"";
    }

    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    std::string buildOptStr = buildOpt.str();
    std::string sourceFileStr = sourceFile.str();
    // No need to pass debugOpt. Already in buildOptStr.
    return buildProgramFromString(program, ocl, prog.c_str(), prog.length(), log, sourceFileStr.c_str(),
                                  buildOptStr.c_str(), nullptr);
  }

  cl_int buildProgramFromString(cl::Program &program, cl::Context &ocl,
                                const char *sourceCode, size_t sourceLength,
                                std::ostream &log,
                                const char *referenceName,
                                const char *buildArgs,
                                const char *debugOption)
  {
    if (sourceLength == 0)
    {
      sourceLength = strlen(sourceCode);
    }

    cl::Program::Sources source(1, std::make_pair(sourceCode, sourceLength + 1));
    cl::Program localProgram(ocl, source);

    // Setup additional build options such as include dirs.
    std::ostringstream buildOpt;
    bool firstOpt = true;

    auto prefixOpt = [&firstOpt](std::ostream &out) -> std::ostream &
    {
      if (!firstOpt)
      {
        out << ' ';
      }
      firstOpt = false;
      return out;
    };

    if (buildArgs && buildArgs[0])
    {
      prefixOpt(buildOpt) << buildArgs;
    }

    if (debugOption && debugOption[0])
    {
      prefixOpt(buildOpt) << debugOption;
    }

    cl_int clerr = localProgram.build(buildOpt.str().c_str());

    if (clerr == CL_SUCCESS)
    {
      program = localProgram;
    }
    else
    {
      log << "Failed to build " << referenceName << ": " << clu::errorCodeString(clerr) << '\n';
      clu::printBuildLogs(log, localProgram, ocl, CL_BUILD_ERROR);
    }

    return clerr;
  }
}
