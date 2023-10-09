////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "RandomDataSource.hpp"

namespace csp::visualquery {

////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string RandomDataSource::sName = "RandomDataSource";

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string RandomDataSource::sSource() {
  return cs::utils::filesystem::loadToString(
      "../share/resources/nodes/csp-visual-query/RandomDataSource.js");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<RandomDataSource> RandomDataSource::sCreate() {
  return std::make_unique<RandomDataSource>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RandomDataSource::RandomDataSource() noexcept
    : mMinLat(-90)
    , mMinLon(-180)
    , mMaxLat(90)
    , mMaxLon(180) {
  std::random_device randomDevice;
  mRandomNumberGenerator = std::mt19937(randomDevice());
  mDistribution          = std::uniform_real_distribution();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RandomDataSource::~RandomDataSource() noexcept = default;

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& RandomDataSource::getName() const noexcept {
  return sName;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RandomDataSource::process() noexcept {
  std::vector<Point2D> points;
  points.reserve(static_cast<size_t>((mMaxLat - mMinLat) * (mMaxLon - mMinLon)));

  for (double lat = mMinLat; lat <= mMaxLat; lat += 1.0) {
    for (double lon = mMinLon; lon <= mMaxLon; lon += 1.0) {
      points.emplace_back(Point2D{static_cast<float>(lon), static_cast<float>(lat),
          std::vector{static_cast<float>(mDistribution(mRandomNumberGenerator))}});
    }
  }

  mData = std::make_shared<Image2D>(points, 0,
      Bound{static_cast<float>(mMinLon), static_cast<float>(mMaxLon)},
      Bound{static_cast<float>(mMinLat), static_cast<float>(mMaxLat)});

  writeOutput("Image2D", mData);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RandomDataSource::onMessageFromJS(nlohmann::json const& message) {
  mMinLat = message.at("minLat");
  mMinLon = message.at("minLon");
  mMaxLat = message.at("maxLat");
  mMaxLon = message.at("maxLon");

  process();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

nlohmann::json RandomDataSource::getData() const {
  return {
      {"minLat", mMinLat},
      {"minLon", mMinLon},
      {"maxLat", mMaxLat},
      {"maxLon", mMinLon},
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void RandomDataSource::setData(nlohmann::json const& json) {
  mMinLat = json.at("minLat");
  mMinLon = json.at("minLon");
  mMaxLat = json.at("maxLat");
  mMaxLon = json.at("maxLon");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::visualquery