////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_ATMOSPHERE_RENDERER_HPP
#define CSP_ATMOSPHERE_RENDERER_HPP

#include "../../../src/cs-scene/CelestialObject.hpp"
#include "Plugin.hpp"

#include <VistaBase/VistaVectorMath.h>
#include <VistaKernel/DisplayManager/VistaViewport.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>
#include <VistaOGLExt/VistaBufferObject.h>
#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaOGLExt/VistaVertexArrayObject.h>

#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>

namespace cs::graphics {
class ShadowMap;
class HDRBuffer;
} // namespace cs::graphics

namespace csp::atmospheres {

struct AtmosphereComponent {
  float                         mBaseDensity{};
  float                         mScaleHeight{}; ///< In Kilometers.
  std::shared_ptr<VistaTexture> mPhaseMap;
  std::shared_ptr<VistaTexture> mExtinctionMap;
};

/// This class draws a configurable atmosphere. Just put an OpenGLNode into your SceneGraph at the
/// very same position as your planet. Set its scale to the same size as your planet.
class AtmosphereRenderer : public IVistaOpenGLDraw {
 public:
  explicit AtmosphereRenderer(std::shared_ptr<Plugin::Settings> settings);

  /// Updates the current sun position and brightness.
  void setSun(glm::vec3 const& direction, float illuminance, float luminance);

  /// Set the planet's radii.
  void setRadii(glm::dvec3 const& radii);

  /// Set the transformation used to draw the atmosphere.
  void setWorldTransform(glm::dmat4 const& transform);

  /// When set, the shader will draw this texture at the given altitude.
  void setClouds(std::string const& textureFile, float height);

  /// When set, the shader will make lookups in order to generate light shafts.
  void setShadowMap(std::shared_ptr<cs::graphics::ShadowMap> const& pShadowMap);

  /// When set, this buffer will be used as background texture instead of the current backbuffer.
  void setHDRBuffer(std::shared_ptr<cs::graphics::HDRBuffer> const& pHDRBuffer);

  /// Returns a value [0..1] which approximates the overall brightness of the atmosphere. Will be
  /// close to zero in outer space or in the planets shadow, close to one on the bright surface of
  /// the planet. It uses the camera data from the last rendering call. Use this value for fake HDR
  /// effects, such as reducing star brightness.
  float getApproximateSceneBrightness() const;

  /// How many samples are taken along the view ray. Trades quality for performance. Default is 15.
  int  getPrimaryRaySteps() const;
  void setPrimaryRaySteps(int iValue);

  /// How many samples are taken along the sun rays. Trades quality for performance. Default is 3.
  int  getSecondaryRaySteps() const;
  void setSecondaryRaySteps(int iValue);

  /// The maximum height of the atmosphere above the planets surface relative to the planets radius.
  float getAtmosphereHeight() const;
  void  setAtmosphereHeight(float dValue);

  /// The different components of the atmosphere.
  std::vector<AtmosphereComponent> const& getAtmosphereComponents() const;
  void setAtmosphereComponents(std::vector<AtmosphereComponent> const& value);

  /// If true, an artificial disc is drawn in the suns direction.
  bool getEnableRefraction() const;
  void setEnableRefraction(bool bEnable);

  /// If true, a ocean is drawn at water level.
  bool getDrawWater() const;
  void setDrawWater(bool bEnable);

  /// The height of the ocean level. In atmosphere space, that means 0 equals sea level, larger
  /// values increase the sea level. If set to AtmosphereHeight, the ocean will be at atmosphere the
  /// boundary.
  float getWaterLevel() const;
  void  setWaterLevel(float fValue);

  /// A value of 0 will multiply the planet surface with the extinction factor of the sun color,
  /// making the night side completely dark. A value of 1 will result in no extinction of the
  /// incoming light.
  float getAmbientBrightness() const;
  void  setAmbientBrightness(float fValue);

  bool Do() override;
  bool GetBoundingBox(VistaBoundingBox& bb) override;

 private:
  void initData();
  void updateShader();

  std::shared_ptr<Plugin::Settings> mPluginSettings;
  std::unique_ptr<VistaTexture>     mCloudTexture;
  std::string                       mCloudTextureFile;
  float                             mCloudHeight    = 0.001F;
  bool                              mUseClouds      = false;
  glm::dvec3                        mRadii          = glm::dvec3(1.0, 1.0, 1.0);
  glm::dmat4                        mWorldTransform = glm::dmat4(1.0);

  std::shared_ptr<cs::graphics::ShadowMap> mShadowMap;
  std::shared_ptr<cs::graphics::HDRBuffer> mHDRBuffer;

  VistaGLSLShader        mAtmoShader;
  VistaVertexArrayObject mQuadVAO;
  VistaBufferObject      mQuadVBO;

  struct GBufferData {
    std::unique_ptr<VistaTexture> mDepthBuffer;
    std::unique_ptr<VistaTexture> mColorBuffer;
  };

  std::unordered_map<VistaViewport*, GBufferData> mGBufferData;

  std::vector<AtmosphereComponent> mComponents;

  bool      mShaderDirty       = true;
  bool      mEnableRefraction  = false;
  bool      mDrawWater         = false;
  float     mWaterLevel        = 0.0F;
  float     mAmbientBrightness = 0.2F;
  float     mAtmosphereHeight  = 1.0F;
  int       mPrimaryRaySteps   = 15;
  int       mSecondaryRaySteps = 4;
  float     mSunIlluminance    = 1.F;
  float     mSunLuminance      = 1.F;
  glm::vec3 mSunDirection      = glm::vec3(1, 0, 0);

  float mApproximateBrightness = 0.0F;

  struct {
    uint32_t sunIlluminance    = 0;
    uint32_t sunLuminance      = 0;
    uint32_t sunDir            = 0;
    uint32_t farClip           = 0;
    uint32_t waterLevel        = 0;
    uint32_t ambientBrightness = 0;
    uint32_t depthBuffer       = 0;
    uint32_t colorBuffer       = 0;
    uint32_t cloudTexture      = 0;
    uint32_t cloudAltitude     = 0;
    uint32_t shadowCascades    = 0;

    uint32_t                baseDensities = 0;
    uint32_t                scaleHeights  = 0;
    std::array<uint32_t, 4> phaseMaps{};
    std::array<uint32_t, 4> extinctionMaps{};

    std::array<uint32_t, 5> shadowMaps{};
    std::array<uint32_t, 5> shadowProjectionMatrices{};

    uint32_t inverseModelViewMatrix    = 0;
    uint32_t inverseProjectionMatrix   = 0;
    uint32_t modelViewMatrix           = 0;
  } mUniforms;

  const char* cAtmosphereVert;
  const char* cAtmosphereFrag;
};

} // namespace csp::atmospheres

#endif // CSP_ATMOSPHERE_RENDERER_HPP
