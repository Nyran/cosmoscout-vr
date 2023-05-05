////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "TileNode.hpp"

namespace csp::lodbodies {

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode()
    : mTileData() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode(TileBase* tile)
    : mTileData(tile)
    , mParent(nullptr) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode::TileNode(std::unique_ptr<TileBase>&& tile)
    : mTileData(std::move(tile))
    , mParent(nullptr) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int TileNode::getLevel() const {
  return mTileData->getLevel();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::int64 TileNode::getPatchIdx() const {
  return mTileData->getPatchIdx();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileId const& TileNode::getTileId() const {
  return mTileData->getTileId();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileBase* TileNode::getTileData() const {
  return mTileData.get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setTileData(std::unique_ptr<TileBase> tile) {
  mTileData = std::move(tile);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileNode::getChild(int childIdx) const {
  return mChildren.at(childIdx).get();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setChild(int childIdx, TileNode* child) {
  // unset OLD parent
  if (mChildren.at(childIdx)) {
    mChildren.at(childIdx)->setParent(nullptr);
  }

  mChildren.at(childIdx).reset(child);

  // set NEW parent
  if (mChildren.at(childIdx)) {
    mChildren.at(childIdx)->setParent(this);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

TileNode* TileNode::getParent() const {
  return mParent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void TileNode::setParent(TileNode* parent) {
  mParent = parent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool TileNode::isRefined() const {
  return mChildren[0] && mChildren[1] && mChildren[2] && mChildren[3];
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::lodbodies
