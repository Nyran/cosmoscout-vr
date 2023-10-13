////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CSP_VISUAL_QUERY_WCS_IMAGE_LOADER_HPP
#define CSP_VISUAL_QUERY_WCS_IMAGE_LOADER_HPP

#include "../../../csl-node-editor/src/Node.hpp"
#include "../../../csl-ogc/src/wcs/WebCoverageService.hpp"
#include "../types/types.hpp"

namespace csp::visualquery {

class WCSImageLoader : public csl::nodeeditor::Node {
 public:
  // static interface ------------------------------------------------------------------------------

  static const std::string          sName;
  static std::string                sSource();
  static std::unique_ptr<WCSImageLoader> sCreate(
    std::shared_ptr<std::vector<csl::ogc::WebCoverageService>> wcs);

  // instance interface ----------------------------------------------------------------------------

  /// New instances of this node are created by the node factory.

  explicit WCSImageLoader(std::shared_ptr<std::vector<csl::ogc::WebCoverageService>> wcsUrl);
  ~WCSImageLoader() override;

  /// Each node must override this. It simply returns the static sName.
  std::string const& getName() const override;

  /// Whenever the simulation time changes, the TimeNode will call this method itself. It simply
  /// updates the value of the 'time' output. This method may also get called occasionally by the
  /// node editor, for example if a new web client was connected hence needs updated values for all
  /// nodes.
  void process() override;

  /// This will be called whenever the CosmoScout.sendMessageToCPP() is called by the JavaScript
  /// client part of this node.
  /// @param message  A JSON object as sent by the JavaScript node. In this case, it is actually
  ///                 just the currently selected server.
  void onMessageFromJS(nlohmann::json const& message) override;

  /// This is called whenever the node needs to be serialized. It returns a JSON object containing
  /// the currently selected server.
  nlohmann::json getData() const override;

  /// This is called whenever the node needs to be deserialized. The given JSON object should
  /// contain a server.
  void setData(nlohmann::json const& json) override;

  void sendServersToJs();
  void sendImageLayersToJs();

 private:
  // Image2D mImage;
  std::shared_ptr<std::vector<csl::ogc::WebCoverageService>> mWcs;
  int mSelectedWcsIndex;
};

} // namespace csp::visualquery

#endif // CSP_VISUAL_QUERY_WCS_IMAGE_LOADER_HPP