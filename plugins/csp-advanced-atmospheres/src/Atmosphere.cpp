////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#include "Atmosphere.hpp"

#include "logger.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-scene/CelestialObject.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <VistaKernel/GraphicsManager/VistaGroupNode.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaKernelOpenSGExt/VistaOpenSGMaterialTools.h>

namespace csp::atmospheres {

////////////////////////////////////////////////////////////////////////////////////////////////////

Atmosphere::Atmosphere(std::shared_ptr<Plugin::Settings> pluginSettings,
    std::shared_ptr<cs::core::Settings>                  settings,
    std::shared_ptr<cs::core::SolarSystem> solarSystem, std::string objectName)
    : mPluginSettings(std::move(pluginSettings))
    , mAllSettings(std::move(settings))
    , mSolarSystem(std::move(solarSystem))
    , mObjectName(std::move(objectName))
    , mEclipseShadowReceiver(
          std::make_shared<cs::core::EclipseShadowReceiver>(mAllSettings, mSolarSystem, false))
    , mRenderer(mPluginSettings, mEclipseShadowReceiver) {

  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  mAtmosphereNode.reset(pSG->NewOpenGLNode(pSG->GetRoot(), &mRenderer));
  mAtmosphereNode->SetIsEnabled(false);
  VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
      mAtmosphereNode.get(), static_cast<int>(cs::utils::DrawOrder::eAtmospheres));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Atmosphere::~Atmosphere() {
  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  pSG->GetRoot()->DisconnectChild(mAtmosphereNode.get());
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::configure(Plugin::Settings::Atmosphere const& settings) {
  auto object = mSolarSystem->getObject(mObjectName);
  if (object) {
    mRenderer.configure(settings, object->getRadii());
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

AtmosphereRenderer& Atmosphere::getRenderer() {
  return mRenderer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::update() {
  auto object = mSolarSystem->getObject(mObjectName);

  if (object && object->getIsBodyVisible() && mPluginSettings->mEnabled.get()) {
    double sunIlluminance = 10.0;

    if (mAllSettings->mGraphics.pEnableHDR.get()) {
      sunIlluminance = mSolarSystem->getSunIlluminance(object->getObserverRelativePosition());
    }

    auto sunDirection = mSolarSystem->getSunDirection(object->getObserverRelativePosition());
    mRenderer.setSun(sunDirection, static_cast<float>(sunIlluminance));
    mRenderer.setWorldTransform(object->getObserverRelativeTransform());
    mEclipseShadowReceiver->update(*object);

    mAtmosphereNode->SetIsEnabled(true);
  } else {
    mAtmosphereNode->SetIsEnabled(false);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
