//
// author Kazys Stepanas
//
#include <ohmapp/OhmAppGpu.h>
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

int main(int argc, char *argv[])
{
  return ohmappMain<ohmapp::OhmAppGpu, ohmapp::SlamIOSource>(argc, argv);
}
