////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CS_AUDIO_FILE_READER_HPP
#define CS_AUDIO_FILE_READER_HPP

#include "cs_audio_export.hpp"


namespace cs::audio {

class CS_AUDIO_EXPORT FileReader {
 public:
  FileReader(const FileReader& obj) = delete;
  FileReader(FileReader&&) = delete;

  FileReader& operator=(const FileReader&) = delete;
  FileReader& operator=(FileReader&&) = delete;

  static bool loadWAV(const char* fn, WavContainer& wavContainer);
  
 private:
  // wave files
  static int convertToInt(char* buffer, int len);
  static bool isBigEndian();
};

} // namespace cs::audio

#endif // CS_AUDIO_FILE_READER_HPP