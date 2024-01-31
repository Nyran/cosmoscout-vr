////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "RealVec2.hpp"

#include "../../../../../src/cs-utils/filesystem.hpp"

namespace csp::visualquery {

////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string RealVec2::sName = "RealVec2";

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string RealVec2::sSource() {
  return cs::utils::filesystem::loadToString(
      "../share/resources/nodes/csp-visual-query/RealVec2.js");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<RealVec2> RealVec2::sCreate() {
  return std::make_unique<RealVec2>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& RealVec2::getName() const {
  return sName;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RealVec2::process() {
  writeOutput("value", mValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RealVec2::onMessageFromJS(nlohmann::json const& message) {
  // The message sent via CosmoScout.sendMessageToCPP() contains the selected number.
  if (message.contains("first")) {
    mValue.first = message["first"];
  } else {
    mValue.second = message["second"];
  }

  // Whenever the user entered a number, we write it to the output socket by calling the process()
  // method. Writing the output will not trigger a graph reprocessing right away, it will only queue
  // up the connected nodes for being processed in the next update step (and only if the value
  // actually changed).
  process();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

nlohmann::json RealVec2::getData() const {
  return {{"value", mValue}};
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RealVec2::setData(nlohmann::json const& json) {
  mValue = json["value"];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::visualquery
