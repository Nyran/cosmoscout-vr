////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "OGCException.hpp"

#include <spdlog/fmt/fmt.h>

namespace csl::ogc {

////////////////////////////////////////////////////////////////////////////////////////////////////

OGCException::OGCException(std::string code, std::string text)
    : mCode(std::move(code))
    , mText(std::move(text))
    , mMessage(fmt::format("{}: {}", mCode, mText)) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& OGCException::getText() const noexcept {
  return mText;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::string const& OGCException::getCode() const noexcept {
  return mCode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

const char* OGCException::what() const noexcept {
  return mMessage.c_str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csl::ogc