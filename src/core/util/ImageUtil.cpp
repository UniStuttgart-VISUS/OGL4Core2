#include "ImageUtil.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include <lodepng.h>

using namespace OGL4Core2::Core;

namespace {
    void flipImage(std::vector<unsigned char>& img, int width, int height, int pixelSize) {
        if (static_cast<int>(img.size()) != width * height * pixelSize) {
            throw std::runtime_error("Invalid image size!");
        }
        int rowSize = width * pixelSize;
        for (int i = 0; i < height / 2; i++) {
            std::swap_ranges(img.begin() + i * rowSize, img.begin() + (i + 1) * rowSize,
                img.begin() + (height - i - 1) * rowSize);
        }
    }
} // namespace

std::vector<unsigned char> ImageUtil::loadPngImage(const std::filesystem::path& filename, int& width, int& height) {
    std::vector<unsigned char> image;
    unsigned int w, h;
    unsigned int error = lodepng::decode(image, w, h, filename.string());
    if (error != 0) {
        std::string errorText = lodepng_error_text(error);
        throw std::runtime_error("Cannot load PNG image: " + errorText);
    }
    width = static_cast<int>(w);
    height = static_cast<int>(h);

    flipImage(image, width, height, 4);

    return image;
}

void ImageUtil::savePngImage(const std::filesystem::path& filename, std::vector<unsigned char>&& image, int width,
    int height) {
    flipImage(image, width, height, 4);

    std::vector<unsigned char> png;
    auto error = lodepng::encode(png, image, static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    if (error == 0) {
        lodepng::save_file(png, filename.string());
    } else {
        std::string errorText = lodepng_error_text(error);
        throw std::runtime_error("Cannot write PNG image: " + errorText);
    }
}
