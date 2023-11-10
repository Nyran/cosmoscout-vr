////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CS_AUDIO_BASE_SOURCE_HPP
#define CS_AUDIO_BASE_SOURCE_HPP

#include "cs_audio_export.hpp"
#include "SourceSettings.hpp"
#include "UpdateInstructor.hpp"

#include <AL/al.h>
#include <map>
#include <any>

namespace cs::audio {

// forward declaration
class SourceGroup;

class CS_AUDIO_EXPORT SourceBase 
  : public SourceSettings
  , public std::enable_shared_from_this<SourceBase> {
    
 public:
  ~SourceBase();
  
  /// @brief Sets setting to start playback
  void play();

  /// @brief Sets setting to stop playback
  void stop();

  /// @brief Sets setting to pause playback
  void pause();

  virtual bool setFile(std::string file) = 0;

  /// @return Returns the current file that is being played by the source.
  std::string getFile() const;

  /// @return Returns to OpenAL ID
  ALuint getOpenAlId() const;

  /// @return Returns all settings (Source + Group + Controller) currently set and playing.
  std::shared_ptr<std::map<std::string, std::any>> getPlaybackSettings() const;

  SourceBase(std::string file, std::shared_ptr<UpdateInstructor> UpdateInstructor);

  // friend class cs::core::AudioEngine;
  friend class SourceGroup;
  friend class UpdateConstructor;
    
 protected:
  ALuint                                           mOpenAlId; 
  /// Currently set file to play
  std::string                                      mFile;
  /// Ptr to the group that the source is part of
  std::shared_ptr<SourceGroup>                     mGroup;
  /// Contains all settings (Source + Group + Controller) currently set and playing. 
  std::shared_ptr<std::map<std::string, std::any>> mPlaybackSettings;

  /// @brief registers itself to the updateInstructor to be updated 
  void addToUpdateList();
};

} // namespace cs::audio

#endif // CS_AUDIO_BASE_SOURCE_HPP
