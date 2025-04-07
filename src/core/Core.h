#pragma once

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

#include "Input.h"
#include "camera/AbstractCamera.h"
#include "util/FpsCounter.h"

namespace OGL4Core2::Core {
    class RenderPlugin;

    class Core {
    public:
        struct Config {
            std::string defaultPluginName;
            std::vector<uint32_t> screenshotFrames;
            std::string screenshotFilename;
            bool autoQuit = false;
        };

        explicit Core(Config cfg);
        ~Core();

        void run();

        [[nodiscard]] std::filesystem::path getPluginResourcesPath() const;

        [[nodiscard]] bool isKeyPressed(Key key) const;
        [[nodiscard]] bool isMouseButtonPressed(MouseButton button) const;
        void getMousePos(double& xpos, double& ypos) const;

        void setWindowSize(int width, int height) const;

        void registerCamera(const std::shared_ptr<AbstractCamera>& camera) const;
        void removeCamera() const;

#ifdef OGL4CORE2_ENABLE_PWROWG
        void push_gaze_point(std::array<float, 2> const& gp);
        std::array<float,2> pull_gaze_point() const;
        bool record_gaze_point() const;
        bool replay_gaze_point() const;
#endif

    private:
        void validateImGuiScale();
        void draw();
        void screenshot();

        void windowSizeEvent(int width, int height);
        void framebufferSizeEvent(int width, int height);
        void keyEvent(int key, int scancode, int action, int mods);
        void charEvent(unsigned int codepoint);
        void mouseButtonEvent(int button, int action, int mods);
        void mouseMoveEvent(double xpos, double ypos);
        void mouseScrollEvent(double xoffset, double yoffset);

        void scaleWindowPosToFramebufferPos(double& xpos, double& ypos) const;

        Config cfg_;

        GLFWwindow* window_;
        bool running_;

        uint64_t frameNumber_;

        FpsCounter fps_;

        std::shared_ptr<RenderPlugin> currentPlugin_;
        std::filesystem::path currentPluginResourcesPath_;
        std::exception currentPluginResourcesPathException_;
        int currentPluginIdx_;
        int pluginSelectionIdx_;
        std::vector<char> pluginNamesImGui_;

        // Only for Win32 and X11 systems window size to framebuffer size is 1:1, according to GLFW doc. Within this
        // application we want to ignore high DPI scaling - at least for now - and handle only one size within the
        // plugins as main coordinate system. Because we are targeting OpenGL development, we use the framebuffer size
        // for this. Nevertheless, GLFW will return the mouse position in window coordinates. Therefore, we need to map
        // this input to the framebuffer coordinate system. This requires to also manage the window size changes.
        int windowWidth_;
        int windowHeight_;
        int framebufferWidth_;
        int framebufferHeight_;
        float contentScale_;
        double mouseX_;
        double mouseY_;

        AbstractCamera::MouseControlMode cameraControlMode_;
        mutable std::weak_ptr<AbstractCamera> camera_;

        static void initGLFW();
        static void terminateGLFW();

        static int glfwReferenceCounter_;

#ifdef OGL4CORE2_ENABLE_PWROWG
        bool run_benchmark_ = false;
        int num_frames_ = 100;
        int expected_samples_num_ = 1;
        int sample_interval_ = 10;
        void* pwr_owg_config_ = nullptr;
        std::string output_file_ = "./pwr.csv";
        std::string camera_path_file_ = "./cam.path";
        std::string eye_path_file_ = "./eye.path";
        bool replay_paths_ = false;
        bool replay_eyes_ = false;
        bool recording_paths_ = false;
        std::array<int, 2> window_size_;
        std::array<int, 2> old_window_size_;
        int frame_cap_ms_ = 30;
        bool use_frame_cap_ = false;

        struct cam_control_entry {
            AbstractCamera::MouseControlMode mode;
            double oldX, oldY, newX, newY, xoffset, yoffset;
        };
        cam_control_entry current_entry_;
        std::vector<cam_control_entry> recorded_entries_;
        std::vector<std::array<float, 2>> recorded_gaze_points_;
        std::array<float, 2> current_gaze_point_ = {0.5f, 0.5f};
#endif
    };
} // namespace OGL4Core2::Core
