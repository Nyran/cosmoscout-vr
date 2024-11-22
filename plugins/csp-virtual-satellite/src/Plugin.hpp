////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CSP_VIRTUAL_SATELLITE_PLUGIN_HPP
#define CSP_VIRTUAL_SATELLITE_PLUGIN_HPP

#include "../../../src/cs-core/PluginBase.hpp"
#include "../../../src/cs-core/Settings.hpp"
#include "RestRequestManager.hpp"

#include <memory>
#include <unordered_map>

namespace csp::virtualsatellite {

class BoxRenderer;

/// Your plugin description here!
class Plugin : public cs::core::PluginBase {
 public:
  struct Settings {
    cs::utils::DefaultProperty<std::string> mUrl{"http://localhost:8000/"};
  };

  void init() override;
  void deInit() override;
  void update() override;

 private:
  void                          onLoad();
  BeanStructuralElementInstance getSEI(std::string const& uuid);
  CategoryAssignment            getCA(std::string const& uuid);

  std::shared_ptr<Settings> mPluginSettings = std::make_shared<Settings>();

  RestRequestManager mManagementAPI;
  RestRequestManager mModelAPI;

  std::string mRepoName;

  std::unique_ptr<BoxRenderer> mBoxRenderer;

  std::unordered_map<std::string, BeanStructuralElementInstance> mSEICache;
  std::unordered_map<std::string, CategoryAssignment>            mCACache;

  int mOnLoadConnection = -1;
  int mOnSaveConnection = -1;
};

} // namespace csp::virtualsatellite

#endif // CSP_VIRTUAL_SATELLITE_PLUGIN_HPP