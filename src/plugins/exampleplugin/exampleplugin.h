#pragma once

#include <string>

#include "core/PluginRegister.h"
#include "core/RenderPlugin.h"

namespace OGL4Core2::Plugins::ExamplePlugin {
    class ExamplePlugin : public Core::RenderPlugin {
        REGISTERPLUGIN(ExamplePlugin, 10) // NOLINT

    public:
        using Core::RenderPlugin::RenderPlugin;

        static std::string name() {
            return "ExamplePlugin";
        }

        void render() override;
    };
} // namespace OGL4Core2::Plugins::ExamplePlugin
