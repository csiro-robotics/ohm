// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

// A very simply utility which performs the probabilty-to-value or reverse conversion used in ohm.

#include <ohm/MapProbability.h>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

#include <iomanip>
#include <limits>

namespace
{
  using real = double;

  struct Options
  {
    real value{ 0 };
    bool reverse{ false };
  };
}  // namespace

int parseOptions(Options *opt, int argc, char *argv[])
{
  cxxopts::Options opt_parse(argv[0], "Make a probability to value conversion or the --reverse .");
  opt_parse.positional_help("<value>");

  try
  {
    // Build GPU options set.
    // clang-format off
    opt_parse.add_options()
      ("help", "Show help.")
      ("v,value", "The input map file (ohm).", cxxopts::value(opt->value))
      ("r,reverse", "Reverse lookup? The input value is treated as a probability and converted to a value.",
               optVal(opt->reverse))
      ;
    // clang-format on

    opt_parse.parse_positional({ "value" });

    cxxopts::ParseResult parsed = opt_parse.parse(argc, argv);

    if (parsed.count("help") || parsed.arguments().empty())
    {
      // show usage.
      std::cout << opt_parse.help() << std::endl;
      return 1;
    }
  }
  catch (const cxxopts::OptionException &e)
  {
    std::cerr << "Argument error\n" << e.what() << std::endl;
    return -1;
  }

  return 0;
}


int main(int argc, char *argv[])
{
  Options opt;
  std::cout.imbue(std::locale(""));

  int res = parseOptions(&opt, argc, argv);

  if (res)
  {
    return res;
  }

  const real converted = (!opt.reverse) ? ohm::probabilityToValue(opt.value) : ohm::valueToProbability(opt.value);
  std::cout << std::fixed << std::setprecision(std::numeric_limits<real>::digits10 + 1) << converted << std::endl;
  return 0;
}
