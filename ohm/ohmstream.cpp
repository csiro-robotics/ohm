// Copyright (c) 2015
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmstream.h"

#ifdef OHM_ZIP
#include <zlib.h>
#endif // OHM_ZIP

#include <cstring>
#include <fstream>
#include <string>

namespace ohm
{
#ifdef OHM_ZIP
  struct Compression
  {
    z_stream stream;
    Byte *buffer;
    unsigned bufferSize;
    bool initialised;

    Compression(unsigned bufferSize = 16 * 1024u);
    ~Compression();

    int initDeflate();
    int doneDeflate();

    int initInflate();
    int doneInflate();
  };


  Compression::Compression(unsigned bufferSize)
    : buffer(nullptr)
    , bufferSize(bufferSize)
    , initialised(false)
  {
    memset(&stream, 0u, sizeof(stream));
    buffer = new Byte[bufferSize];
  }


  Compression::~Compression()
  {
    delete[] buffer;
  }


  int Compression::initDeflate()
  {
    if (!initialised)
    {
      memset(&stream, 0u, sizeof(stream));
      return deflateInit(&stream, Z_DEFAULT_COMPRESSION);
    }
    return Z_OK;
  }


  int Compression::doneDeflate()
  {
    if (initialised)
    {
      initialised = false;
      return deflateEnd(&stream);
    }
    return Z_OK;
  }


  int Compression::initInflate()
  {
    if (!initialised)
    {
      memset(&stream, 0u, sizeof(stream));
      return inflateInit(&stream);
    }
    return Z_OK;
  }


  int Compression::doneInflate()
  {
    if (initialised)
    {
      initialised = false;
      return inflateEnd(&stream);
    }
    return Z_OK;
  }

#endif // OHM_ZIP

  struct StreamPrivate
  {
    std::string filePath;
    unsigned flags;
  };

  struct InputStreamPrivate : StreamPrivate
  {
    std::ifstream in;
#ifdef OHM_ZIP
    Compression compress;
#endif // OHM_ZIP
  };

  struct OutputStreamPrivate : StreamPrivate
  {
    std::ofstream out;
#ifdef OHM_ZIP
    Compression compress;
    bool needsFlush;
#endif // OHM_ZIP
  };
}

using namespace ohm;


const char *Stream::filePath() const
{
  return _imp->filePath.c_str();
}


unsigned Stream::flags() const
{
  return _imp->flags;
}


bool Stream::open(const char *filePath, unsigned flags)
{
  close();
  return doOpen(filePath, flags);
}


void Stream::close()
{
  if (isOpen())
  {
    flush();
    doClose();
  }
}


void Stream::seek(size_t pos)
{
  flush();
  doSeek(pos);
}


size_t Stream::tell()
{
  return doTell();
}


Stream::Stream(StreamPrivate *imp)
  : _imp(imp)
{
  _imp->flags = 0u;
}


Stream::~Stream()
{
}


InputStream::InputStream(const char *filePath, unsigned flags)
  : Stream(new InputStreamPrivate)
{
  if (filePath && filePath[0])
  {
    doOpen(filePath, flags);
  }
}


InputStream::~InputStream()
{
  delete imp();
}


void InputStream::setCompressedFlag()
{
#ifdef OHM_ZIP
  if (!isOpen())
  {
    return;
  }

  unsigned &flags = imp()->flags;
  bool changed = !(flags & SF_Compress);
  flags |= SF_Compress;
  if (changed)
  {
    imp()->compress.initInflate();
  }
#endif // OHM_ZIP
}


unsigned InputStream::read(void *buffer, unsigned maxBytes)
{
#ifdef OHM_ZIP
  if (imp()->flags & SF_Compress)
  {
    InputStreamPrivate &imp = *this->imp();
    int ret;
    unsigned have;

    imp.compress.stream.avail_out = maxBytes;
    imp.compress.stream.next_out = (unsigned char *)(buffer);

    do
    {
      if (!imp.compress.stream.avail_in)
      {
        // No data currently available. Prime the buffer.
        unsigned readBytes = readRaw(imp.compress.buffer, imp.compress.bufferSize);
        imp.compress.stream.avail_in = readBytes;
        imp.compress.stream.next_in = imp.compress.buffer;

        if (!imp.compress.stream.avail_in)
        {
          // Nothing available.
          break;
        }
      }

      ret = inflate(&imp.compress.stream, Z_NO_FLUSH);

      if (ret != Z_OK && ret != Z_STREAM_END)
      {
        return 0;
      }
    }
    while (imp.compress.stream.avail_out);

    have = maxBytes - imp.compress.stream.avail_out;
    return have;
  }
#endif // OHM_ZIP
  return readRaw(buffer, maxBytes);
}


unsigned InputStream::readRaw(void *buffer, unsigned maxBytes)
{
  std::istream &in = imp()->in;
  std::ifstream::pos_type initialPos = in.tellg();
  in.read((char *)buffer, maxBytes);
  std::ifstream::pos_type endPos = in.tellg();
  unsigned read = unsigned(endPos - initialPos);
  return read;
}


bool InputStream::isOpen() const
{
  return imp()->in.is_open();
}


void InputStream::flush()
{
}


bool InputStream::doOpen(const char *filePath, unsigned flags)
{
  imp()->in.open(filePath, std::ios_base::binary);
  imp()->filePath = filePath;
#ifndef OHM_ZIP
  flags &= ~SF_Compress;
#endif // OHM_ZIP
  imp()->flags = flags;
#ifdef OHM_ZIP
  if (flags & SF_Compress)
  {
    imp()->compress.initInflate();
  }
#endif // OHM_ZIP
  return imp()->in.is_open();
}


void InputStream::doClose()
{
#ifdef OHM_ZIP
  if (_imp->flags & SF_Compress)
  {
    imp()->compress.doneInflate();
  }
#endif // OHM_ZIP
  imp()->in.close();
  _imp->flags = 0;
  _imp->filePath = std::string();
}


void InputStream::doSeek(size_t pos)
{
  imp()->in.seekg(pos, std::ios_base::beg);
}


size_t InputStream::doTell()
{
  return imp()->in.tellg();
}


InputStreamPrivate *InputStream::imp()
{
  return static_cast<InputStreamPrivate *>(_imp);
}


const InputStreamPrivate *InputStream::imp() const
{
  return static_cast<const InputStreamPrivate *>(_imp);
}


OutputStream::OutputStream(const char *filePath, unsigned flags)
  : Stream(new OutputStreamPrivate)
{
  if (filePath && filePath[0])
  {
    doOpen(filePath, flags);
  }

#ifdef OHM_ZIP
  imp()->needsFlush = false;
#endif // OHM_ZIP
}


OutputStream::~OutputStream()
{
  close();
  delete imp();
}



unsigned OutputStream::write(const void *buffer, unsigned maxBytes)
{
#ifdef OHM_ZIP
  if (imp()->flags & SF_Compress)
  {
    OutputStreamPrivate &imp = *this->imp();
    int ret;

    imp.compress.stream.next_in = (Bytef *)buffer;
    imp.compress.stream.avail_in = maxBytes;

    if (imp.compress.stream.avail_out == 0)
    {
      imp.compress.stream.avail_out = imp.compress.bufferSize;
      imp.compress.stream.next_out = imp.compress.buffer;
    }

    do
    {
      ret = deflate(&imp.compress.stream, Z_NO_FLUSH);
      if (ret != Z_OK)
      {
        return ~unsigned(0u);
      }
      imp.needsFlush = true;

      if (imp.compress.stream.avail_out == 0)
      {
        imp.out.write((char *)imp.compress.buffer, imp.compress.bufferSize);
        imp.compress.stream.avail_out = imp.compress.bufferSize;
        imp.compress.stream.next_out = imp.compress.buffer;
      }
    }
    while (imp.compress.stream.avail_in);

    return maxBytes;
  }
#endif // OHM_ZIP
  return writeUncompressed(buffer, maxBytes);
}


unsigned OutputStream::writeUncompressed(const void *buffer, unsigned maxBytes)
{
  imp()->out.write((const char *)buffer, maxBytes);
  if (imp()->out.good())
  {
    return maxBytes;
  }

  return 0;
}


bool OutputStream::isOpen() const
{
  return imp()->out.is_open();
}


void OutputStream::flush()
{
  OutputStreamPrivate &imp = *this->imp();
#ifdef OHM_ZIP
  // Finish deflating.
  if (imp.needsFlush && imp.flags & SF_Compress)
  {
    int ret;
    imp.compress.stream.next_in = nullptr;
    imp.compress.stream.avail_in = 0;
    unsigned have = imp.compress.bufferSize - imp.compress.stream.avail_out;
    do
    {
      ret = deflate(&imp.compress.stream, Z_FINISH);
      have = imp.compress.bufferSize - imp.compress.stream.avail_out;
      if (have)
      {
        imp.out.write((char *)imp.compress.buffer, have);
      }

      imp.compress.stream.avail_out = imp.compress.bufferSize;
      imp.compress.stream.next_out = imp.compress.buffer;
    }
    while (ret == Z_OK);

    imp.needsFlush = false;
  }
#endif // OHM_ZIP
  imp.out.flush();
}


bool OutputStream::doOpen(const char *filePath, unsigned flags /*= 0u*/)
{
  imp()->out.open(filePath, std::ios_base::binary);
  imp()->filePath = filePath;
#ifndef OHM_ZIP
  flags &= ~SF_Compress;
#endif // OHM_ZIP
  imp()->flags = flags;
#ifdef OHM_ZIP
  if (flags & SF_Compress)
  {
    imp()->compress.initDeflate();
  }
#endif // OHM_ZIP
  return imp()->out.is_open();
}


void OutputStream::doClose()
{
#ifdef OHM_ZIP
  if (_imp->flags & SF_Compress)
  {
    imp()->compress.doneDeflate();
  }
#endif // OHM_ZIP
  imp()->out.close();
  _imp->flags = 0;
  _imp->filePath = std::string();
}


void OutputStream::doSeek(size_t pos)
{
  imp()->out.seekp(pos, std::ios_base::beg);
}


size_t OutputStream::doTell()
{
  return imp()->out.tellp();
}


OutputStreamPrivate   *OutputStream::imp()
{
  return static_cast<OutputStreamPrivate *>(_imp);
}


const OutputStreamPrivate   *OutputStream::imp() const
{
  return static_cast<const OutputStreamPrivate *>(_imp);
}
