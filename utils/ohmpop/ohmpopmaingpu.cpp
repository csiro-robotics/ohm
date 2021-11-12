//
// author Kazys Stepanas
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmpopmain.inl>

#include "OhmPopGpu.h"

int main(int argc, char *argv[])
{
  return ohmpopMain<OhmPopGpu, ohmapp::SlamIOSource>(argc, argv);
}
