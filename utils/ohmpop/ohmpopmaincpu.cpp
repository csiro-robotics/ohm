//
// author Kazys Stepanas
//
#include <ohmapp/OhmAppCpu.h>
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

int main(int argc, char *argv[])
{
  return ohmappMain<ohmapp::OhmAppCpu, ohmapp::SlamIOSource>(argc, argv);
}
