# Copyright (c) 2021
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230
#
# Author: Kazys Stepanas

# Force GTestConfig.cmake first.
find_package(GTest CONFIG QUIET)

if(NOT GTest_FOUND)
  find_package(GTest QUIET)
endif(NOT GTest_FOUND)

# FindGTest uses different target names to GTestConfig. Alias them if required
if(TARGET GTest::Main)
  # From find module
  set(GTestMain GTest::Main)
  set(GTestGTest GTest::GTest)
elseif(TARGET GTest::gtest_main)
  # From config
  set(GTestMain GTest::gtest_main)
  set(GTestGTest GTest::gtest)
endif()
