////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Atmosphere.hpp"
#include "logger.hpp"

#ifdef _WIN32
#include <Windows.h>
#endif

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/Shadows.hpp"
#include "../../../src/cs-graphics/TextureLoader.hpp"
#include "../../../src/cs-utils/FrameTimings.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <VistaKernel/DisplayManager/VistaDisplayManager.h>
#include <VistaKernel/DisplayManager/VistaViewport.h>
#include <VistaKernel/GraphicsManager/VistaGeometryFactory.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaMath/VistaBoundingBox.h>
#include <VistaOGLExt/Rendering/VistaGeometryData.h>
#include <VistaOGLExt/Rendering/VistaGeometryRenderingCore.h>
#include <VistaOGLExt/VistaOGLUtils.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaTools/tinyXML/tinyxml.h>

#include <glm/gtc/type_ptr.hpp>
#include <utility>

namespace csp::atmospheres {

AtmosphereRenderer::AtmosphereRenderer(std::shared_ptr<Plugin::Settings> settings)
    : mPluginSettings(std::move(settings)) {

  cAtmosphereVert = R"(
  #version 400

  // inputs
  layout(location = 0) in vec2 vPosition;

  // uniforms
  uniform mat4 uMatInvMV;
  uniform mat4 uMatInvP;

  // outputs
  out VaryingStruct {
    vec3 vRayDir;
    vec3 vRayOrigin;
    vec2 vTexcoords;
  } vsOut;

  void main() {
    // get camera position in model space
    vsOut.vRayOrigin = uMatInvMV[3].xyz;

    // get ray direction model space
    mat4 testInvMV = uMatInvMV;
    testInvMV[3] = vec4(0, 0, 0, 1);
    mat4 testInvMVP = testInvMV * uMatInvP;
    vsOut.vRayDir = (testInvMVP * vec4(vPosition, 0, 1)).xyz;

    // for lookups in the depth and color buffers
    vsOut.vTexcoords = vPosition * 0.5 + 0.5;

    // no tranformation here since we draw a full screen quad
    gl_Position = vec4(vPosition, 0, 1);
  }
)";

  cAtmosphereFrag = R"(
  #version 400

  // inputs
  in VaryingStruct {
    vec3 vRayDir;
    vec3 vRayOrigin;
    vec2 vTexcoords;
  } vsIn;

  // uniforms
  #if $HDR_SAMPLES > 0
    uniform sampler2DMS uColorBuffer;
    uniform sampler2DMS uDepthBuffer;
  #else
    uniform sampler2D uColorBuffer;
    uniform sampler2D uDepthBuffer;
  #endif

  uniform sampler2D uCloudTexture;
  uniform mat4      uMatMVP;
  uniform mat4      uMatInvMV;
  uniform mat4      uMatInvP;
  uniform mat4      uMatMV;
  uniform vec3      uSunDir;
  uniform float     uSunIlluminance;
  uniform float     uSunLuminance;
  uniform float     uWaterLevel;
  uniform float     uCloudAltitude;
  uniform float     uAmbientBrightness;
  uniform float     uFarClip;

  // shadow stuff
  uniform sampler2DShadow uShadowMaps[5];
  uniform mat4            uShadowProjectionViewMatrices[5];
  uniform int             uShadowCascades;

  // outputs
  layout(location = 0) out vec3 oColor;

  // constants
  const float PI          = 3.14159265359;
  const float STEP_LENGTH = $BODY_RADIUS * 0.1 / $PRIMARY_RAY_STEPS;
  const int   MAX_SAMPLES = $PRIMARY_RAY_STEPS * 10;

  // Actually only the first $ATMOSPHERE_COMPONENTS of these vec4's are used.
  uniform vec4      uBaseDensities;
  uniform vec4      uScaleHeights;
  uniform sampler1D uPhaseMaps[$ATMOSPHERE_COMPONENTS];
  uniform sampler1D uExtinctionMaps[$ATMOSPHERE_COMPONENTS];

  vec3 heat(float v) {
    float value = 1.0-v;
    return (0.5+0.5*smoothstep(0.0, 0.1, value))*vec3(
          smoothstep(0.5, 0.3, value),
        value < 0.3 ?
          smoothstep(0.0, 0.3, value) :
          smoothstep(1.0, 0.6, value),
          smoothstep(0.4, 0.6, value)
    );
  }

  // compute the density of the atmosphere for a given model space position
  float GetDensity(vec3 vPos) {
    float fHeight = max(0.0, length(vPos) - $BODY_RADIUS);
    return exp(-fHeight/($HEIGHT_ATMO*0.1)) * 1.2;
  }

  // returns the rayleigh density as x component and the mie density as Y
  vec4 GetComponentDensity(vec3 vPos) {
    float fHeight = max(0.0, length(vPos) - $BODY_RADIUS);
    vec4 result = vec4(0.0);

    for (int i=0; i<$ATMOSPHERE_COMPONENTS; ++i) {
      result[i] = exp(-fHeight/(uScaleHeights[i])) * uBaseDensities[i];
    }

    return result;
  }

  // returns the optical depth between two points in model space
  // The ray is defined by its origin and direction. The two points are defined
  // by two T parameters along the ray. Two values are returned, the rayleigh
  // depth and the mie depth.
  vec4 GetComponentOpticalDepth(vec3 vRayOrigin, vec3 vRayDir, float fTStart, float fTEnd) {
    float fStep = (fTEnd - fTStart) / $SECONDARY_RAY_STEPS;
    vec4 sum = vec4(0.0);

    for (int i=0; i<$SECONDARY_RAY_STEPS; i++) {
      float fTCurr = fTStart + (i+0.5)*fStep;
      vec3  vPos = vRayOrigin + vRayDir * fTCurr;

      sum += GetComponentDensity(vPos);
    }

    return sum * fStep;
  }

  // calculates the extinction based on an optical depth
  vec3 GetBeta(sampler1D extinctionMap) {
    // return vec3(0.01, 0.01, 0.01);
    return texture(extinctionMap, 0.0).rgb*0.0001;
  }

  vec3 GetExtinction(vec4 vOpticalDepth) {
    vec3 totalExtinction = vec3(1.0);

    for (int i=0; i<$ATMOSPHERE_COMPONENTS; ++i) {
      totalExtinction *= exp(-GetBeta(uExtinctionMaps[i])*vOpticalDepth[i]);
    }

    return totalExtinction;
  }

  float GetRefractiveIndex(vec3 vPos) {
    float density = GetDensity(vPos);

    float highDensity = 0.00005448;
    float lowDensity = 1.2;

    float highIndex = 1.0;
    float lowIndex = 1 + 0.0003;

    float alpha = (density-lowDensity) / (highDensity - lowDensity); 
    return lowIndex + alpha * (highIndex - lowIndex);
  }

  vec3 GetRefractiveIndexGradient(vec3 pos, float dh) {
    vec3 normal = normalize(pos);
    return normal * (GetRefractiveIndex(pos + normal*dh*0.5)-GetRefractiveIndex(pos - normal*dh*0.5)) / dh;
  }

  // compute intersections with the atmosphere
  // two T parameters are returned -- if no intersection is found, the first will
  // larger than the second
  bool IntersectSphere(vec3 vRayOrigin, vec3 vRayDir, float fRadius, out float t1, out float t2) {
    float b = dot(vRayOrigin, vRayDir);
    float c = dot(vRayOrigin, vRayOrigin) - fRadius*fRadius;
    float fDet = b * b - c;

    // Ray does not actually hit the sphere.
    if (fDet < 0.0) {
      return false;
    }

    fDet = sqrt(fDet);

    // Clamp t1 to ray origin.
    t1 = max(0, -b - fDet);
    
    // This should ususally be > 0, else ...
    t2 = -b + fDet;

    // ... ray does not actually hit the sphere; exit is behind camera.
    if (t2 < 0) {
      return false;
    }

    return true;
  }

  bool IntersectAtmosphere(vec3 vRayOrigin, vec3 vRayDir, out float t1, out float t2) {
    return IntersectSphere(vRayOrigin, vRayDir, $BODY_RADIUS + $HEIGHT_ATMO, t1, t2);
  }

  bool IntersectPlanetsphere(vec3 vRayOrigin, vec3 vRayDir, out float t1, out float t2) {
    return IntersectSphere(vRayOrigin, vRayDir, $BODY_RADIUS, t1, t2);
  }

  // returns the probability of scattering
  vec3 GetPhase(sampler1D phaseMap, float angle) {
    return texture(phaseMap, angle/PI).rgb;
  }

  // Returns the background color at the current pixel. If multisampling is used, we take the
  // average color.
  vec3 GetBackgroundColor(vec2 texcoords) {
    #if $HDR_SAMPLES > 0
      vec3 color = vec3(0.0);
      for (int i = 0; i < $HDR_SAMPLES; ++i) {
        color += texelFetch(uColorBuffer, ivec2(texcoords * textureSize(uColorBuffer)), i).rgb;
      }
      return color / $HDR_SAMPLES;
    #else
      return texture(uColorBuffer, texcoords).rgb;
    #endif
  }

  // Returns the depth at the current pixel. If multisampling is used, we take the minimum depth.
  float GetDepth(vec2 texcoords) {
    #if $HDR_SAMPLES > 0
      float depth = 1.0;
      for (int i = 0; i < $HDR_SAMPLES; ++i) {
        depth = min(depth, texelFetch(uDepthBuffer, ivec2(texcoords *
        textureSize(uDepthBuffer)), i).r);
      }
      return depth;
    #else
      return texture(uDepthBuffer, texcoords).r;
    #endif
  }

  void main() {

    vec3 vRayDir = normalize(vsIn.vRayDir);

    // t1 and t2 are the distances from the ray origin to the intersections with the
    // atmosphere boundary. If the origin is inside the atmosphere, t1 == 0.
    float t1 = 0;
    float t2 = 0;
    if (!IntersectAtmosphere(vsIn.vRayOrigin, vRayDir, t1, t2)) {
      oColor = GetBackgroundColor(vsIn.vTexcoords);
      return;
    }

    vec3 vSamplePos = vsIn.vRayOrigin + t1*vRayDir;
    oColor = vec3(0.0);

    int   samples          = 0;
    
    float pathLength       = 0;
    vec4  pathOpticalDepth = vec4(0.0);

    bool  exitedAtmosphere = false;
    bool  hitPlanet        = false;
    
    while(++samples < MAX_SAMPLES && !exitedAtmosphere && !hitPlanet) {

      // Decrease step length to ensure a maximum density change per step.
      float stepLength = STEP_LENGTH;
      const float maxDensityChange = 0.1;
      while (abs(GetDensity(vSamplePos) - GetDensity(vSamplePos+vRayDir*stepLength)) > maxDensityChange) {
        stepLength /= 2;
      }

      // First check whether this step will exit the atmosphere.
      if (IntersectAtmosphere(vSamplePos, vRayDir, t1, t2) && t2 < stepLength) {
        stepLength = t2;
        exitedAtmosphere = true;
      }

      // Then check whether we hit the planet.
      if (IntersectPlanetsphere(vSamplePos, vRayDir, t1, t2) && t1 < stepLength) {
        stepLength = t1;
        hitPlanet = true;
      }

      pathOpticalDepth += GetComponentOpticalDepth(vSamplePos, vRayDir, 0, stepLength);

      vec3 stepCenter = vSamplePos + 0.5 * stepLength * vRayDir;

      if (!IntersectPlanetsphere(stepCenter, uSunDir, t1, t2)) {

        IntersectAtmosphere(stepCenter, uSunDir, t1, t2);
        float angle    = acos(dot(vRayDir, uSunDir));

        vec3 totalExtinction = vec3(1.0);
        vec4 vOpticalDepthToSun = GetComponentOpticalDepth(stepCenter, uSunDir, 0, t2);

        for (int i=0; i<$ATMOSPHERE_COMPONENTS; ++i) {
          totalExtinction *= GetExtinction(vOpticalDepthToSun+pathOpticalDepth);
        }

        vec4 densities = GetComponentDensity(stepCenter);

        for (int i=0; i<$ATMOSPHERE_COMPONENTS; ++i) {
          oColor += stepLength * totalExtinction * uSunIlluminance *
                    (densities[i] * GetBeta(uExtinctionMaps[i]) * GetPhase(uPhaseMaps[i], angle));
        }
      }

      vSamplePos += stepLength * vRayDir;
      pathLength += stepLength;

      #if $ENABLE_REFRACTION
        float refractiveIndex = GetRefractiveIndex(vSamplePos);
        vec3 dn = GetRefractiveIndexGradient(vSamplePos, STEP_LENGTH*0.1);
        vRayDir = normalize(refractiveIndex * vRayDir + dn * stepLength);
      #endif
    }

    // Now we compute the look-up position in depth and color buffer. If the refracted ray hit the
    // planet, we project the final vSamplePos to screen space. If it did not hit the planet, we
    // assume that it continues until infinity.
    vec4 sampleCoords;
    if (hitPlanet) {
      sampleCoords = uMatMVP * vec4(vSamplePos, 1.0);
    } else {
      sampleCoords = uMatMVP * vec4(vRayDir, 0.0);
    }
    sampleCoords.xy = sampleCoords.xy / sampleCoords.w * 0.5 + 0.5;

    vec3 backgroundColor = GetBackgroundColor(sampleCoords.xy);

    // If refraction is enabled, we can look a little bit 'behind' the horizon. We can identify this
    // area by consulting the depth buffer: If our ray did not hit the planet, but there is
    // something at sampleCoords, we are actually behind the planet. We will use just black as the
    // background color and draw an artificial sun.
    #if $ENABLE_REFRACTION
      if (!hitPlanet && GetDepth(sampleCoords.xy) < 1) {
        float fSunAngle = max(0,dot(vRayDir, uSunDir));
        backgroundColor = vec3(fSunAngle > 0.99999 ? uSunLuminance : 0);
      }
    #endif

    // This is the color extinction for the entire light path through the atmosphere. We will use
    // this to attenuate the color buffer and the direct sun light.
    backgroundColor *= GetExtinction(pathOpticalDepth);

    #if !$ENABLE_HDR
      const float exposure = 0.6;
      const float gamma    = 2.2;
      oColor = clamp(exposure * oColor, 0.0, 1.0);
      oColor = pow(oColor, vec3(1.0 / gamma));
    #endif

    oColor += backgroundColor;

    //oColor += 0.5*heat(float(samples)/MAX_SAMPLES);
    //oColor += 0.5*heat(pathLength);
  }
)";

  initData();

  // scene-wide settings -----------------------------------------------------
  mPluginSettings->mQuality.connectAndTouch([this](int val) { setPrimaryRaySteps(val); });

  mPluginSettings->mEnableWater.connectAndTouch([this](bool val) { setDrawWater(val); });

  mPluginSettings->mEnableRefraction.connectAndTouch(
      [this](bool val) { setEnableRefraction(val); });

  mPluginSettings->mEnableClouds.connectAndTouch([this](bool val) {
    if (mUseClouds != val) {
      mShaderDirty = true;
      mUseClouds   = val;
    }
  });

  mPluginSettings->mWaterLevel.connectAndTouch([this](float val) { setWaterLevel(val / 1000); });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setSun(glm::vec3 const& direction, float illuminance, float luminance) {
  mSunIlluminance = illuminance;
  mSunLuminance   = luminance;
  mSunDirection   = direction;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setRadii(glm::dvec3 const& radii) {
  mRadii = radii;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setWorldTransform(glm::dmat4 const& transform) {
  mWorldTransform = transform;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setClouds(std::string const& textureFile, float height) {

  if (mCloudTextureFile != textureFile) {
    mCloudTextureFile = textureFile;
    mCloudTexture.reset();
    if (!textureFile.empty()) {
      mCloudTexture = cs::graphics::TextureLoader::loadFromFile(textureFile);
    }
    mShaderDirty = true;
    mUseClouds   = mCloudTexture != nullptr;
  }

  mCloudHeight = height;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setShadowMap(std::shared_ptr<cs::graphics::ShadowMap> const& pShadowMap) {
  if (mShadowMap != pShadowMap) {
    mShadowMap   = pShadowMap;
    mShaderDirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setHDRBuffer(std::shared_ptr<cs::graphics::HDRBuffer> const& pHDRBuffer) {
  if (mHDRBuffer != pHDRBuffer) {
    mHDRBuffer   = pHDRBuffer;
    mShaderDirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getApproximateSceneBrightness() const {
  return mApproximateBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int AtmosphereRenderer::getPrimaryRaySteps() const {
  return mPrimaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setPrimaryRaySteps(int iValue) {
  mPrimaryRaySteps = iValue;
  mShaderDirty     = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int AtmosphereRenderer::getSecondaryRaySteps() const {
  return mSecondaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setSecondaryRaySteps(int iValue) {
  mSecondaryRaySteps = iValue;
  mShaderDirty       = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getAtmosphereHeight() const {
  return mAtmosphereHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setAtmosphereHeight(float dValue) {
  mAtmosphereHeight = dValue;
  mShaderDirty      = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<AtmosphereComponent> const& AtmosphereRenderer::getAtmosphereComponents() const {
  return mComponents;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setAtmosphereComponents(std::vector<AtmosphereComponent> const& value) {
  if (value.size() > 4) {
    logger().warn("Only up to four atmosphere components are currently supported.");
  }

  mComponents  = value;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getEnableRefraction() const {
  return mEnableRefraction;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setEnableRefraction(bool bEnable) {
  mEnableRefraction = bEnable;
  mShaderDirty      = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getDrawWater() const {
  return mDrawWater;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setDrawWater(bool bEnable) {
  mDrawWater   = bEnable;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getWaterLevel() const {
  return mWaterLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setWaterLevel(float fValue) {
  mWaterLevel = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getAmbientBrightness() const {
  return mAmbientBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setAmbientBrightness(float fValue) {
  mAmbientBrightness = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::updateShader() {
  mAtmoShader = VistaGLSLShader();

  std::string sVert(cAtmosphereVert);
  std::string sFrag(cAtmosphereFrag);

  cs::utils::replaceString(sFrag, "$PRIMARY_RAY_STEPS", cs::utils::toString(mPrimaryRaySteps));
  cs::utils::replaceString(sFrag, "$SECONDARY_RAY_STEPS", cs::utils::toString(mSecondaryRaySteps));
  cs::utils::replaceString(sFrag, "$HEIGHT_ATMO", cs::utils::toString(mAtmosphereHeight));
  cs::utils::replaceString(sFrag, "$ENABLE_REFRACTION", std::to_string(mEnableRefraction));
  cs::utils::replaceString(sFrag, "$ATMOSPHERE_COMPONENTS", std::to_string(mComponents.size()));
  cs::utils::replaceString(sFrag, "$BODY_RADIUS", std::to_string(mRadii[0]));
  // cs::utils::replaceString(sFrag, "$DRAW_WATER", std::to_string(mDrawWater));
  // cs::utils::replaceString(sFrag, "$USE_SHADOWMAP", std::to_string(mShadowMap != nullptr));
  // cs::utils::replaceString(sFrag, "$USE_CLOUDMAP", std::to_string(mUseClouds && mCloudTexture));
  cs::utils::replaceString(sFrag, "$ENABLE_HDR", std::to_string(mHDRBuffer != nullptr));
  cs::utils::replaceString(sFrag, "$HDR_SAMPLES",
      mHDRBuffer == nullptr ? "0" : std::to_string(mHDRBuffer->getMultiSamples()));

  mAtmoShader.InitVertexShaderFromString(sVert);
  mAtmoShader.InitFragmentShaderFromString(sFrag);

  mAtmoShader.Link();

  mUniforms.sunIlluminance    = mAtmoShader.GetUniformLocation("uSunIlluminance");
  mUniforms.sunLuminance      = mAtmoShader.GetUniformLocation("uSunLuminance");
  mUniforms.sunDir            = mAtmoShader.GetUniformLocation("uSunDir");
  mUniforms.farClip           = mAtmoShader.GetUniformLocation("uFarClip");
  mUniforms.waterLevel        = mAtmoShader.GetUniformLocation("uWaterLevel");
  mUniforms.ambientBrightness = mAtmoShader.GetUniformLocation("uAmbientBrightness");
  mUniforms.depthBuffer       = mAtmoShader.GetUniformLocation("uDepthBuffer");
  mUniforms.colorBuffer       = mAtmoShader.GetUniformLocation("uColorBuffer");
  mUniforms.cloudTexture      = mAtmoShader.GetUniformLocation("uCloudTexture");
  mUniforms.cloudAltitude     = mAtmoShader.GetUniformLocation("uCloudAltitude");
  mUniforms.shadowCascades    = mAtmoShader.GetUniformLocation("uShadowCascades");
  mUniforms.baseDensities     = mAtmoShader.GetUniformLocation("uBaseDensities");
  mUniforms.scaleHeights      = mAtmoShader.GetUniformLocation("uScaleHeights");

  for (size_t i = 0; i < mComponents.size() && i < 4; ++i) {
    mUniforms.phaseMaps[i] = glGetUniformLocation(
        mAtmoShader.GetProgram(), ("uPhaseMaps[" + std::to_string(i) + "]").c_str());
    mUniforms.extinctionMaps[i] = glGetUniformLocation(
        mAtmoShader.GetProgram(), ("uExtinctionMaps[" + std::to_string(i) + "]").c_str());
  }

  for (size_t i = 0; i < 5; ++i) {
    mUniforms.shadowMaps.at(i) = glGetUniformLocation(
        mAtmoShader.GetProgram(), ("uShadowMaps[" + std::to_string(i) + "]").c_str());
    mUniforms.shadowProjectionMatrices.at(i) = glGetUniformLocation(mAtmoShader.GetProgram(),
        ("uShadowProjectionViewMatrices[" + std::to_string(i) + "]").c_str());
  }

  mUniforms.modelViewProjectionMatrix = mAtmoShader.GetUniformLocation("uMatMVP");
  mUniforms.inverseModelViewMatrix    = mAtmoShader.GetUniformLocation("uMatInvMV");
  mUniforms.inverseProjectionMatrix   = mAtmoShader.GetUniformLocation("uMatInvP");
  mUniforms.modelViewMatrix           = mAtmoShader.GetUniformLocation("uMatMV");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::Do() {
  cs::utils::FrameTimings::ScopedTimer timer("Render Atmosphere");

  if (mShaderDirty) {
    updateShader();
    mShaderDirty = false;
  }

  // save current lighting and meterial state of the OpenGL state machine ----
  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glCullFace(GL_FRONT);
  glDepthMask(GL_FALSE);

  // copy depth buffer -------------------------------------------------------
  if (!mHDRBuffer) {
    std::array<GLint, 4> iViewport{};
    glGetIntegerv(GL_VIEWPORT, iViewport.data());

    auto* viewport   = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];

    data.mDepthBuffer->Bind();
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, iViewport.at(0), iViewport.at(1),
        iViewport.at(2), iViewport.at(3), 0);
    data.mColorBuffer->Bind();
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, iViewport.at(0), iViewport.at(1), iViewport.at(2),
        iViewport.at(3), 0);
  }

  // get matrices and related values -----------------------------------------
  std::array<GLfloat, 16> glMatMV{};
  glGetFloatv(GL_MODELVIEW_MATRIX, glMatMV.data());
  glm::mat4 matMV(glm::make_mat4x4(glMatMV.data()) * glm::mat4(mWorldTransform) *
                  glm::mat4(static_cast<float>(mRadii[0] / (mRadii[0])), 0, 0, 0, 0,
                      static_cast<float>(mRadii[1] / (mRadii[0])), 0, 0, 0, 0,
                      static_cast<float>(mRadii[2] / (mRadii[0])), 0, 0, 0, 0, 1));

  auto matInvMV = glm::inverse(matMV);

  std::array<GLfloat, 16> glMatP{};
  glGetFloatv(GL_PROJECTION_MATRIX, glMatP.data());
  glm::mat4 matMVP    = glm::make_mat4x4(glMatP.data()) * matMV;
  glm::mat4 matInvP   = glm::inverse(glm::make_mat4x4(glMatP.data()));
  glm::mat4 matInvMVP = glm::inverse(matMVP);

  glm::vec3 sunDir =
      glm::normalize(glm::vec3(glm::inverse(mWorldTransform) * glm::vec4(mSunDirection, 0)));

  // set uniforms ------------------------------------------------------------
  mAtmoShader.Bind();

  mAtmoShader.SetUniform(mUniforms.sunIlluminance, mSunIlluminance);
  mAtmoShader.SetUniform(mUniforms.sunLuminance, mSunLuminance);
  mAtmoShader.SetUniform(mUniforms.sunDir, sunDir[0], sunDir[1], sunDir[2]);
  mAtmoShader.SetUniform(mUniforms.farClip, cs::utils::getCurrentFarClipDistance());

  mAtmoShader.SetUniform(mUniforms.waterLevel, mWaterLevel);
  mAtmoShader.SetUniform(mUniforms.ambientBrightness, mAmbientBrightness);

  if (mHDRBuffer) {
    mHDRBuffer->doPingPong();
    mHDRBuffer->bind();
    mHDRBuffer->getDepthAttachment()->Bind(GL_TEXTURE0);
    mHDRBuffer->getCurrentReadAttachment()->Bind(GL_TEXTURE1);
  } else {
    auto* viewport   = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];
    data.mDepthBuffer->Bind(GL_TEXTURE0);
    data.mColorBuffer->Bind(GL_TEXTURE1);
  }

  mAtmoShader.SetUniform(mUniforms.depthBuffer, 0);
  mAtmoShader.SetUniform(mUniforms.colorBuffer, 1);

  if (mUseClouds && mCloudTexture) {
    mCloudTexture->Bind(GL_TEXTURE2);
    mAtmoShader.SetUniform(mUniforms.cloudTexture, 2);
    mAtmoShader.SetUniform(mUniforms.cloudAltitude, mCloudHeight);
  }

  if (mShadowMap) {
    int texUnitShadow = 3;
    mAtmoShader.SetUniform(
        mUniforms.shadowCascades, static_cast<int>(mShadowMap->getMaps().size()));
    for (size_t i = 0; i < mShadowMap->getMaps().size(); ++i) {
      mShadowMap->getMaps()[i]->Bind(
          static_cast<GLenum>(GL_TEXTURE0) + texUnitShadow + static_cast<int>(i));
      glUniform1i(mUniforms.shadowMaps.at(i), texUnitShadow + static_cast<int>(i));

      auto mat = mShadowMap->getShadowMatrices()[i];
      glUniformMatrix4fv(mUniforms.shadowProjectionMatrices.at(i), 1, GL_FALSE, mat.GetData());
    }
  }

  int texUnitPhase      = 3 + (mShadowMap ? mShadowMap->getMaps().size() : 0);
  int texUnitExtinction = 3 + (mShadowMap ? mShadowMap->getMaps().size() : 0) + mComponents.size();
  glm::vec4 scaleHeights;
  glm::vec4 baseDensities;
  for (size_t i = 0; i < mComponents.size(); ++i) {
    scaleHeights[i]  = mComponents[i].mScaleHeight;
    baseDensities[i] = mComponents[i].mBaseDensity;

    glUniform1i(mUniforms.phaseMaps[i], texUnitPhase + i);
    glUniform1i(mUniforms.extinctionMaps[i], texUnitExtinction + i);

    mComponents[i].mPhaseMap->Bind(GL_TEXTURE0 + texUnitPhase + i);
    mComponents[i].mExtinctionMap->Bind(GL_TEXTURE0 + texUnitExtinction + i);
  }

  glUniform4fv(mUniforms.scaleHeights, 1, glm::value_ptr(scaleHeights));
  glUniform4fv(mUniforms.baseDensities, 1, glm::value_ptr(baseDensities));

  // Why is there no set uniform for matrices???
  glUniformMatrix4fv(mUniforms.modelViewProjectionMatrix, 1, GL_FALSE, glm::value_ptr(matMVP));
  glUniformMatrix4fv(mUniforms.inverseModelViewMatrix, 1, GL_FALSE, glm::value_ptr(matInvMV));
  glUniformMatrix4fv(mUniforms.inverseProjectionMatrix, 1, GL_FALSE, glm::value_ptr(matInvP));
  glUniformMatrix4fv(mUniforms.modelViewMatrix, 1, GL_FALSE, glm::value_ptr(matMV));

  // draw --------------------------------------------------------------------
  mQuadVAO.Bind();
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  mQuadVAO.Release();

  // clean up ----------------------------------------------------------------

  if (mHDRBuffer) {
    mHDRBuffer->getDepthAttachment()->Unbind(GL_TEXTURE0);
    mHDRBuffer->getCurrentReadAttachment()->Unbind(GL_TEXTURE1);
  } else {
    auto* viewport   = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];
    data.mDepthBuffer->Unbind(GL_TEXTURE0);
    data.mColorBuffer->Unbind(GL_TEXTURE1);
  }

  if (mUseClouds && mCloudTexture) {
    mCloudTexture->Unbind(GL_TEXTURE3);
  }

  mAtmoShader.Release();

  glDepthMask(GL_TRUE);

  glPopAttrib();

  // update brightness value -------------------------------------------------
  // This is a crude approximation of the overall scene brightness due to
  // atmospheric scattering, camera position and the sun's position.
  // It may be used for fake HDR effects such as dimming stars.

  // some required positions and directions
  glm::vec4 temp            = matInvMVP * glm::vec4(0, 0, 0, 1);
  glm::vec3 vCamera         = glm::vec3(temp) / temp[3];
  glm::vec3 vPlanet         = glm::vec3(0, 0, 0);
  glm::vec3 vCameraToPlanet = glm::normalize(vCamera - vPlanet);

  // [planet surface ... 5x atmosphere boundary] -> [0 ... 1]
  float fHeightInAtmosphere = std::min(1.0F,
      std::max(0.0F, (glm::length(vCamera) - (1.F - mAtmosphereHeight)) / (mAtmosphereHeight * 5)));

  // [noon ... midnight] -> [1 ... -1]
  float fDaySide = glm::dot(vCameraToPlanet, sunDir);

  // limit brightness when on night side (also in dusk an dawn time)
  float const exponent       = 50.F;
  float fBrightnessOnSurface = std::pow(std::min(1.F, std::max(0.F, fDaySide + 1.F)), exponent);

  // reduce brightness in outer space
  mApproximateBrightness = (1.F - fHeightInAtmosphere) * fBrightnessOnSurface;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::GetBoundingBox(VistaBoundingBox& bb) {
  auto const extend = static_cast<float>(1.0 / (1.0 - mAtmosphereHeight));

  // Boundingbox is computed by translation an edge points
  std::array<float, 3> const fMin = {-extend, -extend, -extend};
  std::array<float, 3> const fMax = {extend, extend, extend};

  bb.SetBounds(fMin.data(), fMax.data());

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::initData() {
  // create quad -------------------------------------------------------------
  std::array<float, 8> const data{-1, 1, 1, 1, -1, -1, 1, -1};

  mQuadVBO.Bind(GL_ARRAY_BUFFER);
  mQuadVBO.BufferData(data.size() * sizeof(float), data.data(), GL_STATIC_DRAW);
  mQuadVBO.Release();

  // positions
  mQuadVAO.EnableAttributeArray(0);
  mQuadVAO.SpecifyAttributeArrayFloat(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0, &mQuadVBO);

  // create textures ---------------------------------------------------------
  for (auto const& viewport : GetVistaSystem()->GetDisplayManager()->GetViewports()) {
    GBufferData bufferData;

    bufferData.mDepthBuffer = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
    bufferData.mDepthBuffer->Bind();
    bufferData.mDepthBuffer->SetWrapS(GL_CLAMP);
    bufferData.mDepthBuffer->SetWrapT(GL_CLAMP);
    bufferData.mDepthBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mDepthBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mDepthBuffer->Unbind();

    bufferData.mColorBuffer = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
    bufferData.mColorBuffer->Bind();
    bufferData.mColorBuffer->SetWrapS(GL_CLAMP);
    bufferData.mColorBuffer->SetWrapT(GL_CLAMP);
    bufferData.mColorBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mColorBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mColorBuffer->Unbind();

    mGBufferData.emplace(viewport.second, std::move(bufferData));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
