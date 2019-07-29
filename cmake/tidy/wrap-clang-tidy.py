from __future__ import print_function

import argparse
import re
import subprocess
import sys

# sys.stderr.write(str(sys.argv))
# sys.exit(1)

# Command line parser setup.
def setup_args():
    parser = argparse.ArgumentParser(description="")
    # parser.add_argument("csv", type=str, help="CSV file to load")
    parser.add_argument("-clang-tidy-binary", help="Delay before executing the test (seconds)")
    parser.add_argument("-runner-py", help="Python script wrapping clang-tidy with support for multiple jobs. run-clang-tidy.py ships with clang-tidy")
    parser.add_argument("-config-file", help="clang-tidy configuration file. Extracted and passed as the -config argument to clang-tidy.")
    return parser


if __name__ == "__main__":
    arg_parse = setup_args()
    args = arg_parse.parse_known_args(sys.argv[1:])

    tidy_args = []
    if args[0].runner_py:
        tidy_args.append(sys.executable)
        tidy_args.append(args[0].runner_py)
        tidy_args.append('-clang-tidy-binary=' + args[0].clang_tidy_binary)
    else:
        tidy_args.append(args[0].clang_tidy_binary)

    if args[0].config_file:
        # Read the config file to use.
        with open(args[0].config_file) as config_file:
            config = config_file.read()
        tidy_args.append('-config={}'.format(config))

    tidy_args.extend(args[1])
    tidy_args.append('-quiet')
    tidy_args.append('-extra-arg=-fdiagnostics-absolute-paths')

    # Need to filter output from clang-tidy. Warnings are logged on stdout, but we need it on stderr to make it show up in
    # the catkin log. We also need to filter out some stderr messages to do with suppressing warnings, so we don't pollute
    # the log.

    tidy = subprocess.Popen(tidy_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines = True)

    filter_expressions = (
        r'^Skipping .*',
        r'^Suppressed [^\s]+ warnings.*',
        r'^[^\s]+ warning(s)? generated*',
        r'^Use -header-filter=.*',
        r'^' + args[0].clang_tidy_binary + r'.*'
    )

    filter_re_string = ''
    for i in range(len(filter_expressions)):
        if i != 0:
            filter_re_string += '|'
        filter_re_string += '({})'.format(filter_expressions[i])

    filter_regex = re.compile(filter_re_string)

    def pump_output(process, stderr_filter):
        for line in tidy.stdout:
            sys.stderr.write(line)

        for line in tidy.stderr:
            if not stderr_filter.match(line):
                sys.stderr.write(line)

    while tidy.poll() == None:
        pump_output(tidy, filter_regex)
    # Flush final output
    pump_output(tidy, filter_regex)
    sys.exit(tidy.returncode)
