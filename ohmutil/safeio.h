// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef SAFEIO_H_
#define SAFEIO_H_

#include "ohmutilexport.h"

#include <cstdio>

#if !defined(_MSC_VER)

#define sprintf_s snprintf
#define sscanf_s sscanf

int ohmutil_API fopen_s(FILE **file, const char *name, const char *mode);

#endif // !defined(_MSC_VER)

#endif // SAFEIO_H_
