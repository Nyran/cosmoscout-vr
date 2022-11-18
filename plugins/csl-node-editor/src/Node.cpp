////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "Node.hpp"

#include "internal/CommunicationChannel.hpp"

#include <algorithm>

namespace csl::nodeeditor {

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::setID(uint32_t id) {
  mID = id;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t Node::getID() const {
  return mID;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::setPosition(std::array<int32_t, 2> position) {
  mPosition = std::move(position);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::array<int32_t, 2> const& Node::getPosition() const {
  return mPosition;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::setIsCollapsed(bool collapsed) {
  mIsCollapsed = collapsed;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Node::getIsCollapsed() const {
  return mIsCollapsed;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::setSocket(std::shared_ptr<CommunicationChannel> socket) {
  mSocket = std::move(socket);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::setGraph(std::shared_ptr<NodeGraph> graph) {
  mGraph = std::move(graph);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Node::sendMessageToJS(nlohmann::json const& message) const {
  mSocket->sendEvent(CommunicationChannel::Event{
      CommunicationChannel::Event::Type::eNodeMessage, {{"toNode", mID}, {"message", message}}});
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csl::nodeeditor
