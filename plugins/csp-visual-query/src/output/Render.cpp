////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "Render.hpp"

#include "../../../../src/cs-utils/filesystem.hpp"

namespace csp::visualquery {

////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string Render::sName = "Renderer";

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string Render::sSource() {
  return cs::utils::filesystem::loadToString(
      "../share/resources/nodes/csp-visual-query/Render.js");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<Render> Render::sCreate() {
  return std::make_unique<Render>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& Render::getName() const {
  return sName;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Render::process() {

  // Whenever this method is called, we send a message to the JavaScript counterpart of this node.
  // The value is sent as a JSON object
  auto json     = nlohmann::json::object();
  json["value"] = readInput<double>("number", 0.0);
  sendMessageToJS(json);
}


////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::visualquery
