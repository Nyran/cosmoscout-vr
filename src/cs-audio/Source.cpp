////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "Source.hpp"
#include "../cs-core/AudioEngine.hpp"
#include "internal/BufferManager.hpp"
#include "internal/alErrorHandling.hpp"
#include "internal/ProcessingStepsManager.hpp"
#include "internal/SettingsMixer.hpp"

#include <AL/al.h>
#include <map>
#include <any>

namespace cs::audio {

Source::Source(std::shared_ptr<BufferManager> bufferManager, 
  std::shared_ptr<ProcessingStepsManager> processingStepsManager,
  std::string file) 
  : mFile(std::move(file)) 
  , mBufferManager(std::move(bufferManager)) 
  , mCurrentSettings(std::make_shared<std::map<std::string, std::any>>()) 
  , mSettings(std::make_shared<std::map<std::string, std::any>>())
  , mProcessingStepsManager(std::move(processingStepsManager)) {

  alGetError(); // clear error code

  // TODO: check if file actually exists

  // generate new source  
  alGenSources((ALuint)1, &mOpenAlId);
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to generate OpenAL-Soft Source!");
    return;
  }

  // get buffer and bind buffer to source
  alSourcei(mOpenAlId, AL_BUFFER, mBufferManager->getBuffer(mFile));
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to bind buffer to source!");
    return;
  }

  mProcessingStepsManager->process(mOpenAlId, mCurrentSettings);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Source::~Source() {
  alGetError(); // clear error code
  alDeleteSources(1, &mOpenAlId);
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to delete source!");
  }
  mBufferManager->removeBuffer(mFile);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Source::play() const {
  alSourcePlay(mOpenAlId);
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to start playback of source!");
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Source::stop() const {
  alSourceStop(mOpenAlId);
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to stop playback of source!");
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Source::update() {
  // call all processing steps
  mProcessingStepsManager->process(mOpenAlId, mSettings);

  // TODO: ErrorHandling
  SettingsMixer::addSettings(*mCurrentSettings, mSettings);
    
  // reset settings
  mSettings->clear(); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Source::set(std::string key, std::any value) {
  mSettings->operator[](key) = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Source::setFile(std::string file) {
  alGetError(); // clear error code
  // alSourceStop(mOpenAlId);
  alSourcei(mOpenAlId, AL_BUFFER, NULL);
  if (alErrorHandling::errorOccurred()) {
    logger().warn("Failed to remove buffer from source!");
    return false;
  }
  mBufferManager->removeBuffer(mFile);
  
  // TODO: check if file exists
  
  mFile = file;
  mBufferManager->getBuffer(mFile);
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string Source::getFile() const {
  return mFile;   
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<std::map<std::string, std::any>> Source::getSettings() const {
  return mCurrentSettings;
}

} // namespace cs::audio