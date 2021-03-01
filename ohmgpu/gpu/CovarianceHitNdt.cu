// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include <gputil/cuda/cutil_importcl.h>
#include <gputil/gpu_ext.h>

#define NDT 1
#include "CovarianceHitNdt.cl"

GPUTIL_CUDA_DEFINE_KERNEL(covarianceHitNdt);

#undef NDT
#define NDT 2
#include "CovarianceHitNdt.cl"

GPUTIL_CUDA_DEFINE_KERNEL(covarianceHitNdtTm);
