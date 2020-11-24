#!/usr/bin/python3
"""This script wraps up running clang tidy to acheive the goals listed below.

1. Support specifying a yaml configuration file on the command line (-config-file)
2. Optionally run the `run-clang-tidy` python wrapper, which supports job threads
3. Filter redundant output from `run-clang-tidy`
4. Mutate paths to deal with several path reporting issues, which come from how compile_commands.json is configured
5. Optionally report project relative paths.

These goals are described in more detail below.

1. Command line config file

Clang-tidy frustratingly only supports a specific .clang-tidy file which is recursively searched, or command line
cofiguration. Configuration can be very long, so file configuration is preferable, but we want to support different
analysis modes. So we add support to load a .clang-tidy yaml file as specified by `-config-file` and pass this to
clang tidy via the command line.

2. Use `run-clang-tidy`

clang-tidy ships with a companion python script `run-clang-tidy`. This script is currently the only way to run
clang-tidy and use multiple threads. The script essentially splits the files listed and sets up separate processes
for each file.

3. Fix redundant output from `run-clang-tidy`

When using `run-clang-tidy` it outputs the entire configuration used to stdout. This makes for a not of redundant
output especially when processing many files. We filter and suppress this output.

4. Mutate paths

An interesting quirk was discovered in path reporting when using ninja to generate the compile_commands.json. Ninja can
add relative include paths to the json file. This chains on to highlight an undefined behaviour bug in clang-tidy.
Essentially, depending on which checks actually report issues or not, the output can inlcude a mix of absolute and
relative paths - relative to the compile_commands.json file. Specifically, `google-readability-casting` would report
relative paths, unless `readability-identifier-naming` also had issues to report (this is the only known case).

TODO(KS): insert bug report reference

We detect these relative paths, assume they are relative to compile_commands.json and make them absolute.

5. Project relative paths

Project relative paths can be reported by specifying the argument `-relative-to`. Reported paths are made relative to
this path. The intension is to report portable paths which can, for example, be generated on a CI server, but handled
on another computer.
"""
from __future__ import print_function

import argparse
import os
import platform
import re
import subprocess
import sys
import threading

if sys.version_info[0] >= 3:
    from queue import Queue
else:
    from Queue import Queue

# Command line parser setup.


def setup_args():
    """Setup and return the command line argument parser"""
    parser = argparse.ArgumentParser(description='')
    # parser.add_argument('csv', type=str, help='CSV file to load')
    parser.add_argument(
        '-clang-tidy-binary', help='Path to the clang-tidy executable.',  metavar='PATH', required=True)
    parser.add_argument('-clang-apply-replacements-binary',
                        help='Path to the clang-apply-replacements binary. Required when using -fix and -runner-py' +
                        ' arguments.')
    parser.add_argument(
        '-runner-py', help='Python script wrapping clang-tidy with support for multiple jobs. run-clang-tidy.py ships' +
        ' with clang-tidy. Without this clang-tidy is run directly.', metavar='PATH')
    parser.add_argument('-fix', action='store_true',
                        help='Apply automatic fixes. Passes -fix to clang-tidy. When using -runner-py' +
                        ' (run-clang-tidy.py), the argument -clang-apply-replacements-binary must also be set to the' +
                        ' clang-apply-fixes binary.')
    parser.add_argument(
        '-config-file', help='clang-tidy configuration file. Extracted and passed as the -config argument to' +
                             ' clang-tidy.')
    parser.add_argument(
        '-p', help='clang-tidy build path (path to compile_commands.json). Extracted and passed as the -p argument to' +
                   ' clang-tidy.', required=False)
    parser.add_argument(
        '-j', help='Number of parallel jobs to run. Only supported when using the -runner-py script. Ignored ' +
        'otherwise.', required=False)
    parser.add_argument(
        '-relative-to', help='Modify clang-tidy message paths to be relative to this directory. Intended for CI' +
        ' builds to report portable paths.', required=False)
    return parser


def resolve_build_path(args):
    """Resolve the compile_commands.json directory if specified. None when not present."""
    # Use argparser to resolve the -p option so we support '-p <path>" (space)
    # and "-p=<path>" (equals)
    build_path = None
    if args.p is not None:
        # Strip 'comile_commands.json' if present
        build_path = args.p
        if build_path.endswith('compile_commands.json'):
            build_path = build_path[:len(
                build_path) - len('compile_commands.json')]
    return build_path


def escape_path(path):
    # Need to escape back slashes in args for Windows.
    if platform.system() == 'Windows':
        return path.replace('\\', '\\\\')
    return path


class ProcessMessagePump:
    """A helper class for handling subprocess output and attempting to maintain output order.

    Starts a thread each for stdout and stderr then collates on the main thread.

    Usage:
    - Create a subprocess with both stdout and stderr set to subprocess.PIPE
    - Create a ProcessMessagePump around the process
    - Call ProcessMessagePump.pump() with an appropriate logging function.
    """

    def __init__(self, process):
        """Create a piper around process"""
        self.process = process
        self.log_queue = Queue()
        self.pipes_running = 0

    def pump(self, log):
        """Start logging using the log function until the process is done.

        The log function signature must be log(process, pipe, line)
        """
        threading.Thread(target=ProcessMessagePump._pump, args=[
                         self, self.process.stdout]).start()
        threading.Thread(target=ProcessMessagePump._pump, args=[
                         self, self.process.stderr]).start()
        self.pipes_running += 2
        while self.pipes_running > 0 or not self.log_queue.empty():
            pipe_source, line = self.log_queue.get()
            if pipe_source is None or line is None:
                continue
            log(self.process, pipe_source, line)

    def _pump(self, pipe):
        """Thread pump function"""
        try:
            # Keep going until the process ends
            while self.process.poll() is None:
                # Read a line each loop and add to the queue
                line = pipe.readline()
                if line:
                    self.log_queue.put((pipe, line))
            # Final flush
            try:
                for line in iter(pipe.readline, ''):
                    self.log_queue.put((pipe, line))
            except:
                # Ok to have an I/O operation failure on this call. The pipe may have been closed
                pass
        finally:
            # Ensure we note completion of this thread in the queue and class.
            self.log_queue.put((pipe, None))
            self.pipes_running -= 1


if __name__ == '__main__':
    # ---------------------------------------------------------------------------
    # Parse arguments.
    arg_parse = setup_args()
    args = arg_parse.parse_known_args(sys.argv[1:])

    # ---------------------------------------------------------------------------
    # Start building process arguments in tidy_args
    tidy_args = []
    using_runner = False

    # Handle using run-clang-tidy or clang-tidy directly
    if args[0].runner_py:
        using_runner = True
        if platform.system() == 'Windows':
            # The runner will be an executable on platforms *other* than Windows. For Windows, run via python.
            tidy_args.append(sys.executable)
        tidy_args.append(args[0].runner_py)
        if args[0].clang_tidy_binary:
            tidy_args.append('-clang-tidy-binary=' +
                             escape_path(args[0].clang_tidy_binary))
        if args[0].clang_apply_replacements_binary:
            tidy_args.append('-clang-apply-replacements-binary=' +
                             escape_path(args[0].clang_apply_replacements_binary))
        if args[0].j:
            tidy_args.append('-j')
            tidy_args.append(args[0].j)
        else:
            # We explicitly specify the number of jobs to run. The parallel run script fully loads the CPUs when running
            # parallel, so we limit it to keep any UI and OS tasks responsive.
            try:
                import psutil
                job_threads = psutil.cpu_count() - 2
                if job_threads < 2:
                    job_threads = 2
                tidy_args.append('-j')
                tidy_args.append(str(job_threads))
            except ImportError:
                pass

    else:
        tidy_args.append(escape_path(args[0].clang_tidy_binary))

    # Resolve the compile_commands.json path. Note this must be the path to this file, but exclude the file name.
    # This is perculiar to using run-clang-tidy, and clang-tidy itself is ok with the file name.
    build_path = None
    if args[0].p is not None:
        tidy_args.append('-p={}'.format(args[0].p))
        build_path = resolve_build_path(args[0])

    # Apply fixes?
    if args[0].fix:
        tidy_args.append('-fix')

    # Use command line specified .clang-tidy yaml file and extract content to the command line
    config_lines = []
    if args[0].config_file:
        # Read the config file to use.
        with open(args[0].config_file) as config_file:
            config = config_file.read()
            # # Replace line endings with the character sequence '\' 'n' (2 characters) in a way which deals with
            # # any line ending setup.
            # # Replace microsoft line endings
            # config = config.replace('\r\n', '\\n')
            # # Replace MacOS line endings
            # config = config.replace('\r', '\\n')
            # # Replace Unix line endings
            # config = config.replace('\n', '\\n')
        tidy_args.append('-config={}'.format(config))
        # Build the filter for goal "Fix redundant output from `run-clang-tidy`"
        config_lines = config.splitlines() if using_runner else config_lines

    # Add -quiet to suppress a message about which checks are being used (run-clang-tidy)
    tidy_args.append('-quiet')

    # Add other arguments - like the file list
    tidy_args.extend(args[1])

    # ---------------------------------------------------------------------------
    # Setup running the process.

    # Build a regular expression for path fix parsing.
    # Groups:
    # 0: file path (to normalise)
    # 1: line number if column also present
    # 2: column number if 1/2 present, otherwise line number
    # 3: the rest (message)
    error_msg_re = re.compile(r'^(.*?):(.*)')

    def fix_path(line):
        """Fix certain aspects of paths.

        Firstly clang-tidy with ninja generated compile_commands.json can report relative
        paths (to the compile_commands.json path). We ensure these paths are made absolute.

        Secondly we normalise paths.

        Lastly we modify paths to be relative to args.relative_to if specified.
        """
        match = error_msg_re.match(line)
        if match:
            path = match.groups()[0]
            if build_path is not None:
                if not os.path.isabs(path):
                    # Relative path reported. Make relative to compile_commands.json
                    path = os.path.join(build_path, path)
            # Normalise the path.
            path = os.path.abspath(path)
            if args[0].relative_to:
                path = os.path.relpath(path, args[0].relative_to)
            return '{0}:{1}'.format(path, match.groups()[1])
        return line

    def read_output(process, pipe, line):
        """Process output pump and passthrough"""
        # The run-clang-tidy script (which adds threads) does two annoying things - everything is on stdout and
        # it repeats the config in the log.
        # We filter this here, but only if using the runner. Just checking if the line is in the config setup
        # seems to work well.
        out = sys.stderr
        if pipe == process.stdout:
            out = sys.stdout
            if line.strip('\r\n') in config_lines:
                line = None
        if line:
            out.write(fix_path(line))

    # ---------------------------------------------------------------------------
    # Run the process
    error_code = -1
    tidy = subprocess.Popen(tidy_args, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, universal_newlines=True)
    # Setup message pump
    piper = ProcessMessagePump(tidy)
    piper.pump(read_output)
    error_code = tidy.returncode
    sys.exit(error_code)
