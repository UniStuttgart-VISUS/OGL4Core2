#pragma once

#include <filesystem>
#include <vector>

namespace OGL4Core2::Core {
    class ImageUtil {
    public:
        static std::vector<unsigned char> loadPngImage(const std::filesystem::path& filename, int& width, int& height);

        static void savePngImage(const std::filesystem::path& filename, std::vector<unsigned char>&& image, int width,
            int height);
    };
} // namespace OGL4Core2::Core
