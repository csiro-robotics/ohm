"""Convert a text file into a symbol into a symbol contained in a source file.

This can be used to embed a text file into a compiled binary.
"""
import argparse
import copy
import os.path
import re
import sys

if sys.version_info[0] == 2:
    import __future__


#-------------------------------------------------------------------------------
# Argument parsing setup
#-------------------------------------------------------------------------------
class ArgParser(argparse.ArgumentParser):

    def error(self, message):
        if len(sys.argv) > 1:
            sys.stderr.write('error: %s\n' % message)
            self.print_usage()
            sys.exit(1)
        else:
            self.print_help()
        sys.exit(0)


class StyleInfo:

    def __init__(self,
                 file_matches=(),
                 comment_regex=None,
                 include_regex=None,
                 priority=0):
        self.extensions = file_matches
        self.comment_regex = comment_regex
        self.include_regex = include_regex
        self.priority = priority


def buildStyles():
    # Base C-Style comment
    c_style_comment = re.compile(
        r'(^[ \t\r]*\n)|(^[ \t]*//.*\n)|(//.*$)|(/\*.*\*/)', re.M)
    # C/C++ include regular expression
    c_style_include = re.compile(r'^[ \t]*#include[ \t]+[<"](.*)[>"].*$', re.M)
    styles = {
        'none':
        StyleInfo(),
        'text':
        StyleInfo(),
        'c':
        StyleInfo(['c', 'h'], c_style_comment, c_style_include),
        'c#':
        StyleInfo(['cs'], c_style_comment),
        'cmake':
        StyleInfo(['cmake', 'CMakeLists.txt'], re.compile(r'#.*$', re.M)),
        'cpp':
        StyleInfo(['cpp', 'cxx', 'h', 'hpp', 'hxx'],
                  c_style_comment,
                  c_style_include,
                  priority=1),
        'cuda':
        StyleInfo(['cu'], c_style_comment, c_style_include),
        'java':
        StyleInfo(['java'], c_style_comment),
        'opencl':
        StyleInfo(['cl'], c_style_comment, c_style_include),
        'python':
        StyleInfo(['py'], c_style_comment)
    }
    return styles


def str2bool(val):
    if val.lower() in ('yes', 'true', 'on', 'y', '1'):
        return True
    elif val.lower() in ('no', 'false', 'off', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Invalid boolean string: %s' % val)


parser = ArgParser(description = \
"""Convert a text file into a symbol into a symbol contained in a source file.

This can be used to embed a text file into a compiled binary.
""")

# Build styles
text_file_styles = buildStyles()

parser.add_argument('input',
                    metavar='input',
                    type=str,
                    help='Text file to convert into a C++ compilation unit.')
parser.add_argument('output',
                    metavar='output',
                    type=str,
                    help='Base name/path for the output without extension.')

parser.add_argument('--src-ext',
                    type=str,
                    default='cpp',
                    help='File extention for the source file. Default is cpp.')
parser.add_argument('--h-ext',
                    type=str,
                    default='h',
                    help='File extention for the header file.')
parser.add_argument(
    '--mode',
    type=str,
    default='text',
    choices=('binary', 'text'),
    help=
    'Output mode control; text mode writes as readable text, while bytes write as an array of '
    + 'bytes. Bytes are also written if the string content is too long.')
parser.add_argument(
    '--style',
    type=str,
    default='none',
    choices=text_file_styles.keys(),
    help=
    'Explicitly specify the file type. Affects comment and include directive handling. '
    + 'Matched by extension when set to none. Use "text" for no style.')
parser.add_argument(
    '--comment-regex',
    type=str,
    help=
    'Override for the regular expression used to match comments for minification.'
    + 'Applied as a multi-line regular expression.')
parser.add_argument(
    "--name",
    type=str,
    help=
    "Specifies the output variable name. The default is '<filename>Resource' without the path."
)
parser.add_argument(
    '--minify',
    type=str,
    default='off',
    choices=('off', 'on', 'comments', 'whitespace'),
    help='Minification mode.\n' + '\t\toff : leave as is.\n' +
    '\t\ton  : full minification.\n' +
    '\t\tcomments : strip comment strings. See also --comment-style.\n' +
    '\t\twhitespace : strip leading and trailing whitespace.')
parser.add_argument('--no-import',
                    type=str2bool,
                    const=True,
                    nargs='?',
                    help='Do not expand include/import statements?')
parser.add_argument(
    '--include-regex',
    type=str,
    help=
    'Override for the regular expression used to "include" other files. Expression is '
    +
    'replaced by the file identified in capture 1. This file is searched on the -I paths'
)
parser.add_argument(
    '-I',
    type=str,
    dest='include_dirs',
    action='append',
    help=
    'Specifies a path used to resolve include statements. May be specified multiple times.'
)

args = parser.parse_args()


#-------------------------------------------------------------------------------
# Other functions
#-------------------------------------------------------------------------------
def resolveIncludeFilePath(include_file, include_dirs):
    for dir in include_dirs:
        path = os.path.join(dir, include_file)
        # path = path.replace("/", "\\")
        if os.path.exists(path) and os.path.isfile(path):
            return path
    return None


def resolveIncludeDirectives(regex, content, include_dirs):
    directive = regex.search(content)
    haveShownPaths = False
    included_file_paths = []
    while directive:
        include_file_path = resolveIncludeFilePath(directive.group(1),
                                                   include_dirs)
        next_search_offset = directive.start()
        if include_file_path:
            # Load replacement text if we haven't already
            include_content = ''
            # if not include_file_path in included_file_paths:
            with open(include_file_path, 'r') as include_file:
                include_content = include_file.read()
            included_file_paths.append(include_file_path)
            content = content[:directive.start(
            )] + include_content + content[directive.end():]
        else:
            # Failed to resolve.
            if not haveShownPaths:
                haveShownPaths = True
                print("Using include paths:", include_dirs)
            print("Failed to resolve include", directive.group(0), ":",
                  directive.group(1))
            next_search_offset = directive.end()
        directive = regex.search(content, next_search_offset)
    return content


def selectStyle(styles, args):
    if args.style == 'none':
        input_file = os.path.split(args.input)[1]
        input_ext = os.path.splitext(input_file)[1]
        # Strip leading '.' in extention.
        if len(input_ext):
            input_ext = input_ext[1:]
        # Find the appropriate style by extention.
        selected_style = None
        for style_name in styles.keys():
            style = styles[style_name]
            for extention in style.extensions:
                if input_ext == extention or input_file == extention:
                    if selected_style == None or style.priority > selected_style.priority:
                        selected_style = style
                        break
        if not selected_style:
            # Default to text style.
            selected_style = styles['text']
    else:
        selected_style = styles[args.style]

    # Handle overriding comment and include regular expressions.
    if args.comment_regex or args.include_regex:
        selected_style = copy.copy(selected_style)
        if args.comment_regex:
            selected_style.comment_regex = re.compile(args.comment_regex, re.M)
        if args.include_:
            selected_style.include_regex = re.compile(args.include_regex, re.M)

    return selected_style


#-------------------------------------------------------------------------------
# Processing script.
#-------------------------------------------------------------------------------
# Setup output file names
if args.src_ext[0] != '.':
    args.src_ext = '.' + args.src_ext
if args.h_ext[0] != '.':
    args.h_ext = '.' + args.h_ext

args.output_source = os.path.splitext(args.output)[0] + args.src_ext
args.output_header = os.path.splitext(args.output)[0] + args.h_ext

# Add in the input file directory to the search paths.
if not args.include_dirs:
    args.include_dirs = []
args.include_dirs.append(os.path.split(os.path.abspath(args.input))[0])

# Strip quotes from include_dirs in case they are passed with quotes.
strip_quotes_rex = re.compile(r'(^")|("$)')
for i in range(len(args.include_dirs)):
    args.include_dirs[i] = strip_quotes_rex.sub('', args.include_dirs[i])

# Setup comment regular expression.
style = selectStyle(text_file_styles, args)

# Define the header guard for the output file.
header_guard = os.path.splitext(os.path.split(args.output)[1])[0]
# Replace invalid characters.
resource_name = re.sub(r'[-\+\*\@\!\.]', '_', header_guard)
header_guard = resource_name.upper() + '_H'

if args.name:
    resource_name = args.name

# Write header file
with open(args.output_header, 'w') as header_file:
    header_file.write("""// Resource file generated from {0}
#ifndef {1}
#define {1}

extern const unsigned {2}_length; // NOLINT
extern const char * const {2}; // NOLINT

#endif // {1}
""".format(args.input, header_guard, resource_name))

# Read the file content.
with open(args.input, 'r') as input_file:
    input_file_content = input_file.read()

# Find include directives
if style.include_regex and not args.no_import:
    input_file_content = resolveIncludeDirectives(style.include_regex,
                                                  input_file_content,
                                                  args.include_dirs)

# Minify if required
minify_whitepace = args.minify == 'whitespace'
minify_comments = args.minify == 'comments'
if args.minify == 'on':
    minify_whitepace = True
    minify_comments = True

# Strip comments
if minify_comments and style.comment_regex != None:
    input_file_content = style.comment_regex.sub('', input_file_content)
if minify_whitepace:
    # Remove trailing whitespace
    input_file_content = re.sub(r'[ \t]+$', '', input_file_content, flags=re.M)
    # Replace leading whitespace
    input_file_content = re.sub(r'(^[ \t]+)',
                                '',
                                input_file_content,
                                flags=re.M)

input_file_content_length = len(input_file_content)

# Verify the content length: C++ string declarations are limited to less than 2^16 characters.
# If ok, we append quotes.
if input_file_content_length < 1 << 16 and args.mode == 'text':
    # Build the content string to write
    # Do content replacements:
    #   \           : \\
    #   <newline>   : \n\"<newline>\"
    #   <tab>       : \t
    # Replace quotes.
    input_file_content_string = input_file_content.replace('"', '\\"')
    # Replace backslashes excluding the quoted we just escaped.
    input_file_content_string = re.sub(r'\\(?!")', r'\\\\',
                                       input_file_content_string)
    input_file_content_string = re.sub(r'\r?\n', r'\\n"\n"',
                                       input_file_content_string)
    input_file_content_string = re.sub(r'\t', '\\t', input_file_content_string)
    input_file_content_string = '"' + input_file_content_string + '"'
else:
    # String too long. Convert the original content to byte array.
    input_file_content_bytes = []
    input_file_content_bytes = input_file_content.encode('utf8')
    input_file_content_length = len(input_file_content_bytes)
    input_file_content_string = ''
    content_buffer = ['{\n']
    buffered_count = 0
    for byte in input_file_content_bytes:
        if sys.version_info < (3, 0):
            byte = ord(byte)
        content_buffer.append(str(byte))
        content_buffer.append(',')
        buffered_count += 2
        if buffered_count % 110 == 0:
            content_buffer.append('\n')
    content_buffer.append('\n0\n}')
    input_file_content_string = ''.join(content_buffer)

# Start writing output
with open(args.output_source, 'w') as source_file:
    source_file.write("""// Resource file generated from {0}
#include "{1}"

const unsigned {2}_length = {3};  // NOLINT
static const char {2}_[] =  // NOLINT\n""".format(
        args.input, os.path.basename(args.output_header), resource_name,
        input_file_content_length))

    source_file.write(input_file_content_string)
    source_file.write('; // NOLINT\n')
    source_file.write(
        "const char * const {0} = {0}_; // NOLINT\n".format(resource_name))
    source_file.write('\n')
