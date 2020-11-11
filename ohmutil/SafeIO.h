// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_SAFEIO_H
#define OHMUTIL_SAFEIO_H

#include "OhmUtilExport.h"

#include <cstdio>

#if !defined(_MSC_VER)

#define sprintf_s snprintf
#define sscanf_s sscanf

int ohmutil_API fopen_s(FILE **file, const char *name, const char *mode);  // NOLINT

#endif  // !defined(_MSC_VER)

#endif  // OHMUTIL_SAFEIO_H
