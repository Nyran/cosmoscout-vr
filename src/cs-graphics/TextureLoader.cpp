////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TextureLoader.hpp"

#include "logger.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image.h>
#include <stb_image_resize.h>
#include <stb_image_write.h>
#undef STB_IMAGE_IMPLEMENTATION
#undef STB_IMAGE_WRITE_IMPLEMENTATION
#undef STB_IMAGE_RESIZE_IMPLEMENTATION

#include <VistaOGLExt/VistaOGLUtils.h>
#include <iostream>
#include <tiffio.h>
#include <vector>

namespace cs::graphics {

////////////////////////////////////////////////////////////////////////////////////////////////////

std::unique_ptr<VistaTexture> TextureLoader::loadFromFile(
    std::string const& sFileName, bool generateMipMaps) {

  std::string suffix = sFileName.substr(sFileName.rfind('.'));

  if (suffix == ".tga") {
    // load with vista
    logger().debug("Loading Texture '{}' with Vista.", sFileName);
    return std::unique_ptr<VistaTexture>(VistaOGLUtils::LoadTextureFromTga(sFileName));
  }

  if (suffix == ".tiff" || suffix == ".tif") {
    // load with tifflib
    logger().debug("Loading Texture '{}' with libtiff.", sFileName);

    auto* data = TIFFOpen(sFileName.c_str(), "r");
    if (!data) {
      logger().error("Failed to load '{}' with libtiff!", sFileName);
      return nullptr;
    }

    uint32 width{};
    uint32 height{};
    TIFFGetField(data, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(data, TIFFTAG_IMAGEWIDTH, &width);

    uint16 bpp{};
    TIFFGetField(data, TIFFTAG_BITSPERSAMPLE, &bpp);

    int16 channels{};
    TIFFGetField(data, TIFFTAG_SAMPLESPERPIXEL, &channels);

    if (bpp != 8) {
      logger().error(
          "Failed to load '{}' with libtiff: Only 8 bit per sample are supported right now!",
          sFileName);
      return nullptr;
    }

    std::vector<char> pixels(width * height * channels);

    for (unsigned y = 0; y < height; y++) {
      TIFFReadScanline(data, &pixels[width * channels * y], y);
    }

    GLenum ePixelFormat = GL_RGBA;

    if (channels == 1) {
      ePixelFormat = GL_RED;
    } else if (channels == 2) {
      ePixelFormat = GL_RG;
    } else if (channels == 3) {
      ePixelFormat = GL_RGB;
    }

    std::unique_ptr<VistaTexture> texture;
    uint32                        minDim = std::min(width, height);
    uint32                        maxDim = std::max(width, height);

    if (minDim == 1) {
      texture = std::make_unique<VistaTexture>(GL_TEXTURE_1D);
      texture->UploadTexture(maxDim, 1, pixels.data(), generateMipMaps, ePixelFormat);
    } else {
      texture = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
      texture->UploadTexture(width, height, pixels.data(), generateMipMaps, ePixelFormat);
    }

    TIFFClose(data);

    return texture;

  } else if (suffix == ".hdr") {

    // load with stb image
    logger().debug("Loading HDR Texture '{}' with stbi.", sFileName);

    int width{};
    int height{};
    int bpp{};
    int channels = 4;

    float* pixels = stbi_loadf(sFileName.c_str(), &width, &height, &bpp, channels);

    if (!pixels) {
      logger().error("Failed to load '{}' with stbi!", sFileName);
      return nullptr;
    }

    std::unique_ptr<VistaTexture> texture;
    uint32                        minDim = std::min(width, height);
    uint32                        maxDim = std::max(width, height);

    if (minDim == 1) {
      texture = std::make_unique<VistaTexture>(GL_TEXTURE_1D);
      texture->Bind();
      if (generateMipMaps) {
        gluBuild1DMipmaps(GL_TEXTURE_1D, GL_RGBA32F, maxDim, GL_RGBA, GL_FLOAT, pixels);
      } else {
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA32F, maxDim, 0, GL_RGBA, GL_FLOAT, pixels);
      }
    } else {
      texture = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
      texture->Bind();
      if (generateMipMaps) {
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA32F, width, height, GL_RGBA, GL_FLOAT, pixels);
      } else {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, pixels);
      }
    }

    stbi_image_free(pixels);

    return texture;
  }

  // load with stb image
  logger().debug("Loading Texture '{}' with stbi.", sFileName);

  int width{};
  int height{};
  int bpp{};
  int channels = 4;

  unsigned char* pixels = stbi_load(sFileName.c_str(), &width, &height, &bpp, channels);

  if (!pixels) {
    logger().error("Failed to load '{}' with stbi!", sFileName);
    return nullptr;
  }

  std::unique_ptr<VistaTexture> texture;
  uint32                        minDim = std::min(width, height);
  uint32                        maxDim = std::max(width, height);

  if (minDim == 1) {
    texture = std::make_unique<VistaTexture>(GL_TEXTURE_1D);
    texture->UploadTexture(maxDim, 1, pixels, generateMipMaps);
  } else {
    texture = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
    texture->UploadTexture(width, height, pixels, generateMipMaps);
  }

  stbi_image_free(pixels);

  return texture;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::graphics
