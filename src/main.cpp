#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <cxxopts.hpp>

#include "core/Core.h"

int main(int argc, char* argv[]) {
    cxxopts::Options options("OGL4Core2", "OGL4Core2");
    // clang-format off
    options.add_options()
        ("p,plugin", "Default loaded plugin.", cxxopts::value<std::string>())
        ("s,screenshot", "List of frame numbers for screenshots.", cxxopts::value<std::vector<uint32_t>>())
        ("f,filename", "Base filename for screenshots.", cxxopts::value<std::string>())
        ("q,quit", "Quit when screenshot list is empty.")
        ("h,help", "Show help.");
    // clang-format on

    OGL4Core2::Core::Core::Config cfg;
    cxxopts::ParseResult result;
    try {
        result = options.parse(argc, argv);
        if (result.count("plugin")) {
            cfg.defaultPluginName = result["plugin"].as<std::string>();
        }
        if (result.count("screenshot")) {
            cfg.screenshotFrames = result["screenshot"].as<std::vector<uint32_t>>();
        }
        if (result.count("filename")) {
            cfg.screenshotFilename = result["filename"].as<std::string>();
        }
        if (result.count("quit")) {
            cfg.autoQuit = result["quit"].as<bool>();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error parsing options: " << ex.what() << std::endl;
        std::cerr << options.help() << std::endl;
        return -1;
    }

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    try {
        OGL4Core2::Core::Core c(std::move(cfg));
        c.run();
    } catch (const std::exception& ex) {
        std::cerr << "OGL4Core2 Exception: " << ex.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "OGL4Core2 Exception: Unknown error!" << std::endl;
        return -1;
    }
    return 0;
}
