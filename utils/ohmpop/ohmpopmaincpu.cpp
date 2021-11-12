//
// author Kazys Stepanas
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmpopmain.inl>

#include "OhmPopCpu.h"

int main(int argc, char *argv[])
{
  return ohmpopMain<OhmPopCpu, ohmapp::SlamIOSource>(argc, argv);
}
