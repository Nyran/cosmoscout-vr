////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CS_AUDIO_AUDIO_CONTROLLER_HPP
#define CS_AUDIO_AUDIO_CONTROLLER_HPP

#include "cs_audio_export.hpp"
#include "Source.hpp"
#include "SourceGroup.hpp"
#include "internal/BufferManager.hpp"
#include "internal/ProcessingStepsManager.hpp"

#include <memory>
#include <map>
#include <any>
#include <string>

namespace cs::audio {

class CS_AUDIO_EXPORT AudioController : public SourceSettings {
 public:
  AudioController(
    std::shared_ptr<BufferManager> bufferManager, 
    std::shared_ptr<ProcessingStepsManager> processingStepsManager,
    std::vector<std::string> processingStpes,
    int audioControllerId);

  /// Creates a new audio source
  std::shared_ptr<Source> createSource(std::string file);
  /// Creates a new audio source group
  std::shared_ptr<SourceGroup> createSourceGroup();
  /// Set settings that will be applied to all sources of a plugin 
  void set(std::string key, std::any value);

 private:
  std::shared_ptr<BufferManager>                   mBufferManager;
  std::shared_ptr<ProcessingStepsManager>          mProcessingStepsManager;
  std::shared_ptr<std::map<std::string, std::any>> mGlobalPluginSettings;
  int                                              mAudioControllerId;
};

} // namespace cs::audio

#endif // CS_AUDIO_AUDIO_CONTROLLER_HPP
