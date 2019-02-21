
__kernel void matrixMultiply(__global float *out, __global float *a, __global float *b, unsigned n, __local float *work)
{
  if (get_global_id(0) >= n)
  {
    return;
  }

  for (unsigned i = 0; i < n; ++i)
  {
    work[get_local_id(0)] = a[i * n + get_local_id(0)] * b[get_local_id(0) * n + i];
    barrier(CLK_LOCAL_MEM_FENCE);

    if (get_local_id(0) == 0)
    {
      float r = 0;
      for (unsigned j = 0; j < n; ++j)
      {
        r += work[j];
      }
      out[i] = r;
    }
  }
}
