# Copyright (c) 2017
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Configure variables and include packages.

# Find packages
find_package(Threads)
find_package(glm REQUIRED)

if(OHM_FEATURE_THREADS)
  find_package(TBB CONFIG)
endif(OHM_FEATURE_THREADS)

if(NOT OHM_BUILD_SHARED)
  # We only need to propagate these packages for static ohm builds.
  if(OHM_TES_DEBUG)
    find_package(3es)
  endif(OHM_TES_DEBUG)

  if(OHM_FEATURE_PDAL)
    find_package(PDAL REQUIRED)
  endif(OHM_FEATURE_PDAL)

  find_package(ZLIB)
endif(NOT OHM_BUILD_SHARED)
