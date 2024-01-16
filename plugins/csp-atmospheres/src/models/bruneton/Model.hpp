////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

// SPDX-FileCopyrightText: German Aerospace Center (DLR) <cosmoscout@dlr.de>
// SPDX-License-Identifier: MIT

#ifndef CSP_ATMOSPHERES_MODELS_BRUNETON_MODEL_HPP
#define CSP_ATMOSPHERES_MODELS_BRUNETON_MODEL_HPP

#include "../../../../src/cs-core/Settings.hpp"
#include "../../ModelBase.hpp"
#include "internal/Model.hpp"

namespace csp::atmospheres::models::bruneton {

/// This atmospheric model uses an implementation of multi-scattering by Eric Bruneton. More
/// information can be found in the repo
/// https://github.com/ebruneton/precomputed_atmospheric_scattering as well as in the paper
/// "Precomputed Atmospheric Scattering" (https://hal.inria.fr/inria-00288758/en).
class Model : public ModelBase {
 public:
  struct Settings {
    struct ScatteringComponent {
      std::string mBetaSca;
      std::string mBetaAbs;
      std::string mPhase;
      std::string mDensity;
    };

    struct AbsorbingComponent {
      std::string mBetaAbs;
      std::string mDensity;
    };

    ScatteringComponent               mMolecules;
    ScatteringComponent               mAerosols;
    std::optional<AbsorbingComponent> mOzone;

    // The angular radius of the Sun needs to be specified. As SPICE is not fully available when th
    // plugin is loaded, we cannot compute it. Also, this actually varies in reality.
    double mSunAngularRadius = 0.004675;

    // The average reflectance of the ground used during multiple scattering.
    cs::utils::DefaultProperty<double> mGroundAlbedo{0.1};

    // The number of multiple scattering events to precompute. Use zero for single-scattering only.
    cs::utils::DefaultProperty<int32_t> mMultiScatteringOrder{4};

    // The number of samples to evaluate when pre-computing the optical depth.
    cs::utils::DefaultProperty<int32_t> mSampleCountOpticalDepth{500};

    // The number of samples to evaluate when pre-computing the single scattering. Larger values
    // improves the sampling of thin atmospheric layers.
    cs::utils::DefaultProperty<int32_t> mSampleCountSingleScattering{50};

    // The number of samples to evaluate when pre-computing the multiple scattering. Larger values
    // tend to darken the horizon for thick atmospheres.
    cs::utils::DefaultProperty<int32_t> mSampleCountMultiScattering{50};

    // The number of samples to evaluate when pre-computing the scattering density. Larger values
    // spread out colors in the sky.
    cs::utils::DefaultProperty<int32_t> mSampleCountScatteringDensity{16};

    // The number of samples to evaluate when pre-computing the indirect irradiance.
    cs::utils::DefaultProperty<int32_t> mSampleCountIndirectIrradiance{32};

    // The resolution of the transmittance texture. Larger values can improve the sampling of thin
    // atmospheric layers close to the horizon.
    cs::utils::DefaultProperty<int32_t> mTransmittanceTextureWidth{256};
    cs::utils::DefaultProperty<int32_t> mTransmittanceTextureHeight{64};

    // Larger values improve sampling of thick low-altitude layers.
    cs::utils::DefaultProperty<int32_t> mScatteringTextureRSize{32};

    // Larger values reduce circular banding artifacts around zenith for thick atmospheres.
    cs::utils::DefaultProperty<int32_t> mScatteringTextureMuSize{128};

    // Larger values reduce banding in the day-night transition when seen from space.
    cs::utils::DefaultProperty<int32_t> mScatteringTextureMuSSize{32};

    // Larger values reduce circular banding artifacts around sun for thick atmospheres.
    cs::utils::DefaultProperty<int32_t> mScatteringTextureNuSize{8};

    // The resolution of the irradiance texture.
    cs::utils::DefaultProperty<int32_t> mIrradianceTextureWidth{64};
    cs::utils::DefaultProperty<int32_t> mIrradianceTextureHeight{16};
  };

  /// Whenever the model parameters are changed, this method needs to be called. It will return true
  /// if the shader needed to be recompiled. If that's the case, you can retrieve the new shader
  /// with the getShader() method below.
  bool init(
      nlohmann::json const& modelSettings, double planetRadius, double atmosphereRadius) override;

  /// Returns a fragment shader which you can link to your shader program. See the ModelBase class
  /// for more details. You have to call init() for accessing the shader.
  GLuint getShader() const override;

  /// This model sets three texture uniforms. So it will return startTextureUnit + 3.
  GLuint setUniforms(GLuint program, GLuint startTextureUnit) const override;

 private:
  std::unique_ptr<internal::Model> mModel;
};

} // namespace csp::atmospheres::models::bruneton

#endif // CSP_ATMOSPHERES_MODELS_BRUNETON_MODEL_HPP
