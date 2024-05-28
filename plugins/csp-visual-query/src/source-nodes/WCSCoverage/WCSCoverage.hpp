////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CSP_VISUAL_QUERY_WCS_COVERAGE_HPP
#define CSP_VISUAL_QUERY_WCS_COVERAGE_HPP

#include "../../../../csl-node-editor/src/Node.hpp"
#include "../../../../csl-ogc/src/wcs/WebCoverageService.hpp"
#include "../../../../csl-ogc/src/wcs/WebCoverageTextureLoader.hpp"
#include "../../types/CoverageContainer.hpp"
#include "../../types/types.hpp"

namespace csp::visualquery {

class WCSCoverage : public csl::nodeeditor::Node {
 public:
  // static interface ------------------------------------------------------------------------------

  static const std::string            sName;
  static std::string                  sSource();
  static std::unique_ptr<WCSCoverage> sCreate(std::vector<csl::ogc::WebCoverageService> wcs);

  // instance interface ----------------------------------------------------------------------------

  /// New instances of this node are created by the node factory.

  explicit WCSCoverage(std::vector<csl::ogc::WebCoverageService> wcsUrl);
  ~WCSCoverage() override;

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

  void sendImageChannelsToJs();
  void init() override;

 private:
  // Image2D mImage;
  std::vector<csl::ogc::WebCoverageService>     mWcs;
  std::shared_ptr<csl::ogc::WebCoverageService> mSelectedServer;
  std::shared_ptr<csl::ogc::WebCoverage>        mSelectedImageChannel;
};

} // namespace csp::visualquery

#endif // CSP_VISUAL_QUERY_WCS_COVERAGE_HPP