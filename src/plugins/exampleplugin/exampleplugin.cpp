#include "exampleplugin.h"

#include <glad/gl.h>
#include <imgui.h>

using namespace OGL4Core2::Plugins::ExamplePlugin;

void ExamplePlugin::render() {
    if (ImGui::CollapsingHeader("ExamplePlugin", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Gray screen.");
    }

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}
