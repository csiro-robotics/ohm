# Copyright (c) 2017
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Configure variables and incldue packages.
set(OHM_TES_DEBUG @OHM_TES_DEBUG@)

# Find packages
if(OHM_TES_DEBUG)
  find_package(3es)
endif(OHM_TES_DEBUG)
find_package(Threads)

