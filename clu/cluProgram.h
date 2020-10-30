//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2016
//
#ifndef CLUPROGRAM_H
#define CLUPROGRAM_H

#include "clu.h"

#include <functional>
#include <string>
#include <vector>

namespace clu
{
using LocalMemCalcFunc = std::function<size_t(size_t)>;

/// Output the build log to @p out.
///
/// Logging is filtered based on the build status when @p filterStatus is a valid
/// @c cl_build_status value. Otherwise any status passes.
void printBuildLog(std::ostream &out, const cl::Program &program, const cl::Device &device, int filter_status = 1);

/// Output the build logs to @p out.
///
/// Outputs the log for all devices associated with @p context.
///
/// Logging is filtered based on the build status when @p filterStatus is a valid
/// @c cl_build_status value. Otherwise any status passes.
void printBuildLogs(std::ostream &out, const cl::Program &program, const cl::Context &context, int filter_status = 1);

/// List the devices associated with the given program.
/// @param devices Populated with the devices from @p context.
/// @param program The program to list devices for.
/// @return The number of items in @p devices.
unsigned listDevices(std::vector<cl::Device> &devices,  // NOLINT(google-runtime-references)
                     const cl::Program &program);

/// Calculate the maximum work group size a @p kernel.
///
/// If @p localMemFunc is provide, then the calculated size considers local memory constraints
/// as well. The function must accept a desired work group size and return the amount
/// of local memory required by the kernel given that work group size. The return value
/// is in bytes.
///
/// The function iterates the devices available for the program associated with kernel to find
/// a "valid" result. This behaviour was instigated after simply taking the first device
/// would generate a @c CL_INVALID_DEVICE error code from @c clGetKernelWorkGroupInfo().
/// Iterating the devices avoided the issue.
///
/// There may also be some perculiarities with some CPU devices. It has been noted that the Apple
/// CPU implementation (on Intel) will start to report a @c CL_KERNEL_WORK_GROUP_SIZE from
/// @c clGetKernelWorkGroupInfo() as soon as the kernel includes a call to @c barrier(), either
/// local or global. This is suggestive of lacking any work item synchronisation, forcing all
/// work groups to have one item under these conditions.
///
/// @param kernel The kernel of interest.
/// @param local_mem_func Optional function to calculate Local memory required based on
///     work groups size.
/// @param err Error code return.
/// @return The maximum work group size.
size_t maxWorkgroupSize(const cl::Kernel &kernel, const LocalMemCalcFunc &local_mem_func, cl_int *err);

/// @overload
inline size_t maxWorkgroupSize(const cl::Kernel &kernel, cl_int *err = nullptr)
{
  return maxWorkgroupSize(kernel, LocalMemCalcFunc(), err);
}

/// Resolves the directory in which the application executable lies.
/// This is not the same as the current working directory.
///
/// The directory is written to @p buffer and the number of characters required
/// for the path returned. The @p buffer may be null, in which case the return
/// value can still be used to determine the buffer size (add one for null terminator).
/// @param buffer Populated with the application directory. May be null to get the required size.
/// @param buffer_size Size of @p buffer in bytes.
/// @return The string length of the application directory, excluding the null terminator.
size_t applicationDirectory(char *buffer, size_t buffer_size);

/// A helper function for getting the current working directory.
/// @param cwd The buffer to output to.
/// @param buffer_length The capacity of @p cwd in characters.
/// @return @c cwd
char *currentWorkingDirectory(char *cwd, size_t buffer_length);

/// Get the path separator character for the current platform.
/// @return Either '/' or '\\' (Windows only).
int pathSeparator();

/// Find the directory containing the OpenCL program @p fileName either int he current
/// working directory (preferred) or the application executable directory.
///
/// The @c fileName buffer is modified to the directory containing the original name.
///
/// @param file_name The target file name.
/// @param buffer_length The buffer capacity of @p fileName (in characters).
/// @param search_paths Additional search paths in a comma separated list.
/// @return A pointer to @c fileName on success, null on failure.
char *findProgramDir(char *file_name, size_t buffer_length, const char *search_paths = nullptr);

/// Similar to @c fineProgramDir(), except that the @c fileName is preserved as part of
/// the modified path.
///
/// @param file_name The target file name.
/// @param buffer_length The buffer capacity of @p fileName (in characters).
/// @param search_paths Additional search paths in a comma separated list.
/// @return A pointer to @c fileName on success, or null on failure.
char *findProgramPath(char *file_name, size_t buffer_length, const char *search_paths = nullptr);

/// Build the program from @p sourceFileName found either in the current working
/// directory (preferred), or in the application executable directory.
///
/// This attempts to build the source file @p sourceFileName into @p program using the
/// given @p ocl context. Any @p args are appended to the build options. On success,
/// the @p program is initialised for use and @p sourceFileName is modified to refer
/// to the full path from which the source was loaded. The latter is actually set as
/// soon as the location is resolved even should building fail.
///
/// The @p sourceFileName is search for in the following locations:
/// -# The current working directory
/// -# The current application directory.
/// -# The comma separated paths listed in @p serachPaths.
///
/// The @p sourceOption is intended to identify the build option used to specify the
/// full source file path to the compiler. For instance, this may be used in debugging
/// Intel OpenCL by passing "-g -s", in which case "-g -s &lt;sourceFileName&gt;" is
/// included as the first build enabling debugging and setting the source file reference.
///
/// Any errors are logged to @p log.
///
/// The function simply returns @c CL_SUCCESS if the program is already loaded.
///
/// This function is inline to avoid ABI issues using std library objects.
///
/// @param program The program option to build into.
/// @param ocl The full OpenCL context to build using.
/// @param source_file_name Source file to build from.
/// @param args Additional arguments string to pass to @c cl::Program::build().
/// @param log All errors are logged here including compile errors and warnings.
/// @param debug_option Debug option to be added and postfixed with the full source file name.
/// @param source_file_opt Compiler option used to specify the path to the source file.
///     Source file path is not passed if this is null. Warning: no spaces are inserted
///     between the argument and the source file name. The caller must do so if required.
/// @param search_paths Additional search paths for the @p source_file_name. Use comma as a path separator.
cl_int buildProgramFromFile(cl::Program &program, cl::Context &ocl,  // NOLINT(google-runtime-references)
                            std::string &source_file_name,           // NOLINT(google-runtime-references)
                            std::ostream &log, const char *args = nullptr, const char *debug_option = nullptr,
                            const char *source_file_opt = nullptr, const char *search_paths = nullptr);

cl_int buildProgramFromString(cl::Program &program, cl::Context &ocl,  // NOLINT(google-runtime-references)
                              const char *source_code, size_t source_length, std::ostream &log,
                              const char *reference_name, const char *build_args = nullptr,
                              const char *debug_option = nullptr);
}  // namespace clu

#endif  // CLUPROGRAM_H
