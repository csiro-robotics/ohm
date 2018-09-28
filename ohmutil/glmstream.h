// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GLMSTREAM_H_
#define GLMSTREAM_H_

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

template <typename T>
inline void parseVector(std::istream &in, T *v, int elements)
{
  // Expecting:
  //  <value>,<value>[,...]

  char ch;
  bool brackets = false;
  if (in.peek() == '(')
  {
    brackets = true;
    in >> ch;
  }
  for (int i = 0; i < elements; ++i)
  {
    if (i != 0)
    {
      in >> ch;
      if (ch != ',')
      {
        in.setstate(std::ios::badbit | std::ios::failbit);
      }
    }
    in >> v[i];
  }
  if (brackets)
  {
    in >> ch;
    if (ch != ')')
    {
      in.setstate(std::ios::badbit | std::ios::failbit);
    }
  }
}


inline void parseVector(std::istream &in, char *v, int elements)
{
  int vv[32];
  if (elements > 32)
  {
    in.setstate(std::ios::badbit);
  }
  parseVector(in, vv, elements);
  for (int i = 0; i < elements; ++i)
  {
    v[i] = (char)vv[i];
  }
}


inline void parseVector(std::istream &in, unsigned char *v, int elements)
{
  int vv[32];
  if (elements > 32)
  {
    in.setstate(std::ios::badbit);
  }
  parseVector(in, vv, elements);
  for (int i = 0; i < elements; ++i)
  {
    v[i] = (char)vv[i];
  }
}


template <typename T, glm::precision P>
inline std::istream &operator>>(std::istream &in, glm::tvec2<T, P> &value)
{
  parseVector(in, glm::value_ptr(value), 2);
  return in;
}


template <typename T, glm::precision P>
inline std::istream &operator>>(std::istream &in, glm::tvec3<T, P> &value)
{
  parseVector(in, glm::value_ptr(value), 3);
  return in;
}


template <typename T, glm::precision P>
inline std::istream &operator>>(std::istream &in, glm::tvec4<T, P> &value)
{
  parseVector(in, glm::value_ptr(value), 4);
  return in;
}

#endif // GLMSTREAM_H_
