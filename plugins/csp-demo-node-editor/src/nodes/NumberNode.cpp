////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "NumberNode.hpp"

#include "../../../../src/cs-utils/filesystem.hpp"

namespace csp::demonodeeditor {

////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string NumberNode::sName = "Number";

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string NumberNode::sSource() {
  return cs::utils::filesystem::loadToString(
      "../share/resources/nodes/csp-demo-node-editor/NumberNode.js");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<NumberNode> NumberNode::sCreate() {
  return std::make_unique<NumberNode>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& NumberNode::getName() const {
  return sName;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NumberNode::process() {
  writeOutput("value", mValue);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NumberNode::onMessageFromJS(nlohmann::json const& message) {

  // The CosmoScout.sendMessageToCPP() method sends the current value.
  mValue = message;

  // Whenever the value, we write it to the output socket by calling the process() method.
  // Writing the output will not trigger a graph reprocessing right away, it will only queue
  // up the connected nodes for being processed in the next update step.
  process();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

nlohmann::json NumberNode::getData() const {
  return {{"value", mValue}};
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NumberNode::setData(nlohmann::json const& json) {
  mValue = json["value"];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::demonodeeditor
