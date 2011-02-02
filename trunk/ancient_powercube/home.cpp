#include "ancient_powercube/ancient_powercube.h"
#include <cstdlib>
#include <cstdio>

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    printf("syntax: home PORT BAUD\n");
    return 1;
  }
  AncientPowercube pc(argv[1], atoi(argv[2]));
  return 0;
}
