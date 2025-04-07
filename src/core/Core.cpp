#include "Core.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <glad/gl.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "PluginDescriptor.h"
#include "PluginRegister.h"
#include "RenderPlugin.h"
#include "util/FileUtil.h"
#include "util/GLFWUtil.h"
#include "util/GLUtil.h"
#include "util/ImageUtil.h"

#ifdef OGL4CORE2_ENABLE_PWROWG
#include <fstream>
#include <visus/pwrowg/csv_column.h>
#include <visus/pwrowg/csv_iomanip.h>
#include <visus/pwrowg/csv_sink.h>
#include <visus/pwrowg/emi_configuration.h>
#include <visus/pwrowg/hmc8015_configuration.h>
#include <visus/pwrowg/msr_configuration.h>
#include <visus/pwrowg/nvml_configuration.h>
#include <visus/pwrowg/sensor_array.h>
#include <visus/pwrowg/sensor_array_configuration.h>
#include <visus/pwrowg/sensor_filters.h>
#include <visus/pwrowg/tinkerforge_configuration.h>

/**
* Convert a wide char string to std::string.
* @param name Wide char string.
* @return Char string.
*/
std::string unmueller_string(wchar_t const* name) {
    // https://en.cppreference.com/w/cpp/locale/codecvt/out

    auto const& f = std::use_facet<std::codecvt<wchar_t, char, std::mbstate_t>>(std::locale());

    std::wstring internal(name);
    std::mbstate_t mb = std::mbstate_t();
    std::string external(internal.size() * f.max_length(), '\0');
    const wchar_t* from_next;
    char* to_next;

    auto const res = f.out(mb, &internal[0], &internal[internal.size()], from_next, &external[0],
        &external[external.size()], to_next);
    if (res != std::codecvt_base::ok) {
        throw std::runtime_error("could not convert string");
    }
    external.resize(to_next - &external[0]);

    return external;
}

using stream_type = std::ofstream;
using sink_type = visus::pwrowg::atomic_sink<visus::pwrowg::csv_sink<stream_type>>;

struct pwr_owg_config_t {
    visus::pwrowg::sensor_array sensors;

    std::vector<std::vector<std::pair<std::uint64_t, float>>> sensor_buffers;
    std::vector<std::string> sensor_names;

    std::unique_ptr<sink_type> sink;
};

static void sample_func(visus::pwrowg::sample const* samples, std::size_t cnt,
    visus::pwrowg::sensor_description const* sensors, void* ctx) {
    auto& bufs = reinterpret_cast<pwr_owg_config_t*>(ctx)->sensor_buffers;
    for (std::size_t i = 0; i < cnt; ++i) {
        bufs[samples[i].source].push_back(
            std::pair<uint64_t, float>{samples[i].timestamp.value(), samples[i].reading.floating_point});
    }
}

std::unique_ptr<sink_type> generate_sink(std::filesystem::path const& filepath) {
    stream_type stream;
    stream.open(filepath);
    if (!stream.is_open()) {
        std::cout << "failed to open output stream" << std::endl;
    }
    stream << visus::pwrowg::setcsvdelimiter(';');
    stream << visus::pwrowg::setcsvcolumns(
        visus::pwrowg::csv_column::id | visus::pwrowg::csv_column::label | visus::pwrowg::csv_column::name);
    visus::pwrowg::csvheader(stream);

    return std::make_unique<sink_type>(std::chrono::milliseconds(10), std::move(stream));
}
#endif

using namespace OGL4Core2::Core;

static constexpr int initWindowSizeWidth = 1280;
static constexpr int initWindowSizeHeight = 800;
static constexpr int openGLVersionMajor = 4;
static constexpr int openGLVersionMinor = 5;
static constexpr char imguiGlslVersion[] = "#version 450";
static constexpr char title[] = "OGL4Core2";

Core::Core(Config cfg)
    : cfg_(std::move(cfg)),
      window_(nullptr),
      running_(false),
      frameNumber_(0),
      currentPlugin_(nullptr),
      currentPluginIdx_(-1),
      pluginSelectionIdx_(0),
      windowWidth_(-1),
      windowHeight_(-1),
      framebufferWidth_(-1),
      framebufferHeight_(-1),
      contentScale_(-1.0f),
      mouseX_(0.0),
      mouseY_(0.0),
      cameraControlMode_(AbstractCamera::MouseControlMode::None)
#ifdef OGL4CORE2_ENABLE_PWROWG
      ,
      window_size_{initWindowSizeWidth, initWindowSizeHeight},
      old_window_size_{0, 0}
#endif
{
#ifdef OGL4CORE2_ENABLE_PWROWG
    try {
        pwr_owg_config_ = new pwr_owg_config_t;

        reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sink = generate_sink("./pwr.csv");

        visus::pwrowg::sensor_array_configuration config;
        config.sample_every(std::chrono::milliseconds(sample_interval_))
            .configure<visus::pwrowg::tinkerforge_configuration>([](visus::pwrowg::tinkerforge_configuration& c) {
                c.averaging(visus::pwrowg::tinkerforge_sample_averaging::average_of_4)
                    .voltage_conversion_time(visus::pwrowg::tinkerforge_conversion_time::milliseconds_1_1)
                    .current_conversion_time(visus::pwrowg::tinkerforge_conversion_time::milliseconds_1_1);
            })
            .configure<visus::pwrowg::nvml_configuration>([](visus::pwrowg::nvml_configuration& c) {})
            .exclude<visus::pwrowg::hmc8015_configuration>()
            .exclude<visus::pwrowg::emi_configuration>()
            .exclude<visus::pwrowg::msr_configuration>()
            .deliver_context(reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sink.get())
            .deliver_to(sink_type::sample_callback);

        reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensors =
            visus::pwrowg::sensor_array::for_matches(std::move(config), visus::pwrowg::is_power_sensor);

        auto& sensors = reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensors;
        auto& names = reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensor_names;
        names.clear();
        names.reserve(sensors.size());

        for (int i = 0; i < sensors.size(); ++i) {
            auto& s = sensors[i];
            if (visus::pwrowg::is_tinkerforge_sensor(s)) {
                if (unmueller_string(s.path()) == "UgH") {
                    s.label("HPWR0");
                }
                if (unmueller_string(s.path()) == "Ugu") {
                    s.label("HPWR1");
                }
                if (unmueller_string(s.path()) == "Ufm") {
                    s.label("HPWR2");
                }
                if (unmueller_string(s.path()) == "UgF") {
                    s.label("HPWR3");
                }
                if (unmueller_string(s.path()) == "Uft") {
                    s.label("HPWR4");
                }
                if (unmueller_string(s.path()) == "UeW") {
                    s.label("HPWR5");
                }
                if (unmueller_string(s.path()) == "UfN") {
                    s.label("PEG3V");
                }
                if (unmueller_string(s.path()) == "U6Q") {
                    s.label("PEG5V");
                }
            }
            names.push_back(unmueller_string(s.id()) + "%" + unmueller_string(s.name()));
        }
    } catch (...) {
        std::cout << "failed to configure PWROWG" << std::endl;
        if (pwr_owg_config_) {
            delete pwr_owg_config_;
            pwr_owg_config_ = nullptr;
        }
    }
#endif

    Core::initGLFW();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, openGLVersionMajor);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, openGLVersionMinor);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    window_ = glfwCreateWindow(initWindowSizeWidth, initWindowSizeHeight, title, nullptr, nullptr);
    if (!window_) {
        Core::terminateGLFW();
        throw std::runtime_error("GLFW window creation failed!");
    }

    glfwMakeContextCurrent(window_);

    int gladGLVersion = gladLoadGL(glfwGetProcAddress);
    if (gladGLVersion == 0) {
        throw std::runtime_error("Failed to initialize OpenGL context!");
    }

    std::cout << title << std::endl;
    GLUtil::printOpenGLInfo();

    if (gladGLVersion < GLAD_MAKE_VERSION(openGLVersionMajor, openGLVersionMinor)) {
        throw std::runtime_error("OpenGL context does not match requested version!");
    }

    // Set OpenGL error callback
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(GLUtil::OpenGLMessageCallback, nullptr);
    // ignore notifications
    glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);

    // The initial size above is only a hint for the window manager, but no guarantied window size. Further the window
    // size can be adjusted by DPI scaling on some systems. This initial resize will not be caught by the callback
    // events. Therefore, here do an initial size query.
    glfwGetWindowSize(window_, &windowWidth_, &windowHeight_);
    glfwGetFramebufferSize(window_, &framebufferWidth_, &framebufferHeight_);

    glfwSetWindowUserPointer(window_, this);

    glfwSetWindowRefreshCallback(window_, [](GLFWwindow* window) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->draw();
        glfwSwapBuffers(window);
    });

    // Map callbacks to core class methods
    glfwSetWindowSizeCallback(window_, [](GLFWwindow* window, int width, int height) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->windowSizeEvent(width, height);
    });
    glfwSetFramebufferSizeCallback(window_, [](GLFWwindow* window, int width, int height) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->framebufferSizeEvent(width, height);
    });
    glfwSetKeyCallback(window_, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->keyEvent(key, scancode, action, mods);
    });
    glfwSetCharCallback(window_, [](GLFWwindow* window, unsigned int codepoint) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->charEvent(codepoint);
    });
    glfwSetMouseButtonCallback(window_, [](GLFWwindow* window, int button, int action, int mods) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->mouseButtonEvent(button, action, mods);
    });
    glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double xpos, double ypos) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->mouseMoveEvent(xpos, ypos);
    });
    glfwSetScrollCallback(window_, [](GLFWwindow* window, double xoffset, double yoffset) {
        static_cast<Core*>(glfwGetWindowUserPointer(window))->mouseScrollEvent(xoffset, yoffset);
    });

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(imguiGlslVersion);

    validateImGuiScale();

    // Setup Plugins
    if (PluginRegister::empty()) {
        throw std::runtime_error("No plugins found!");
    }
    // Plugin names for ImGui combo box
    for (const auto& pluginDescriptor : PluginRegister::getAll()) {
        const auto& name = pluginDescriptor->name();
        pluginNamesImGui_.insert(pluginNamesImGui_.end(), name.begin(), name.end());
        pluginNamesImGui_.push_back('\0');
    }
    pluginNamesImGui_.push_back('\0');

    // Plugins will be initialized on the fly in render method. No need to duplicate initialization here.

    // Find default plugin by name
    if (!cfg_.defaultPluginName.empty()) {
        const auto& plugins = PluginRegister::getAll();
        for (std::size_t i = 0; i < plugins.size(); i++) {
            if (cfg_.defaultPluginName == plugins[i]->name()) {
                pluginSelectionIdx_ = static_cast<int>(i);
                break;
            }
        }
    }

    // Sort and filter screenshot frame list
    if (!cfg_.screenshotFrames.empty()) {
        std::sort(cfg_.screenshotFrames.begin(), cfg_.screenshotFrames.end());
        auto last = std::unique(cfg_.screenshotFrames.begin(), cfg_.screenshotFrames.end());
        cfg_.screenshotFrames.erase(last, cfg_.screenshotFrames.end());
        while (!cfg_.screenshotFrames.empty() && cfg_.screenshotFrames.front() < 1) {
            cfg_.screenshotFrames.erase(cfg_.screenshotFrames.begin());
        }
    }
}

Core::~Core() {
    // Delete active plugin here, before destroying the OpenGL context.
    camera_.reset();
    currentPlugin_ = nullptr;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    Core::terminateGLFW();

#ifdef OGL4CORE2_ENABLE_PWROWG
    reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sink->dispose();
    if (pwr_owg_config_) {
        delete pwr_owg_config_;
    }
#endif
}

void Core::run() {
    if (running_) {
        throw std::runtime_error("Core is already running!");
    }
    running_ = true;
    auto render_frame = [&]() {
        frameNumber_++;

        if (fps_.tick()) {
            std::string windowTitle = std::string(title) + " [ " + fps_.getFpsString() + " ]";
            glfwSetWindowTitle(window_, windowTitle.c_str());
        }

        draw();

        screenshot();

        glfwSwapBuffers(window_);
        glfwPollEvents();
    };
    while (!glfwWindowShouldClose(window_)) {
#ifdef OGL4CORE2_ENABLE_PWROWG
        if (recording_paths_) {
            recorded_entries_.push_back(current_entry_);
            current_entry_.newX = current_entry_.oldX;
            current_entry_.newY = current_entry_.oldY;
            current_entry_.xoffset = 0;
            current_entry_.yoffset = 0;
        }
        if (run_benchmark_) {
            try {
                std::cout << "started benchmark" << std::endl;
                reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sink = generate_sink(output_file_);
                reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)
                    ->sensors.start(sink_type::sample_callback,
                        reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sink.get());
                auto iter_time = std::chrono::steady_clock::now();
                auto const start = std::chrono::steady_clock::now();
                if (replay_paths_) {
                    for (std::size_t i = 0; i < recorded_entries_.size(); ++i) {
                        auto const& ce = recorded_entries_[i];
                        if (!recorded_gaze_points_.empty()) {
                            current_gaze_point_ = recorded_gaze_points_[i];
                        }
                        auto camera = camera_.lock();
                        if (camera) {
                            camera->mouseMoveControl(ce.mode, ce.oldX, ce.oldY, ce.newX, ce.newY);
                            camera->mouseScrollControl(ce.xoffset, ce.yoffset);
                        }
                        render_frame();
                        if (use_frame_cap_) {
                            std::this_thread::sleep_until(iter_time + std::chrono::milliseconds(frame_cap_ms_));
                            iter_time = std::chrono::steady_clock::now();
                        }
                    }
                    replay_paths_ = false;
                } else {
                    for (int i = 0; i < num_frames_; ++i) {
                        render_frame();
                        if (use_frame_cap_) {
                            std::this_thread::sleep_until(iter_time + std::chrono::milliseconds(frame_cap_ms_));
                            iter_time = std::chrono::steady_clock::now();
                        }
                    }
                }
                auto const end = std::chrono::steady_clock::now();
                std::cout << "time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                          << " num frames: " << recorded_entries_.size() << std::endl;
                reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensors.stop();
                run_benchmark_ = false;
                auto const& bufs = reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensor_buffers;
                if (std::any_of(bufs.begin(), bufs.end(),
                        [&](auto const& b) { return b.size() >= expected_samples_num_; })) {
                    std::cout << "exceded sample buffer allocation" << std::endl;
                }
                std::cout << "finished benchmark" << std::endl;
            } catch (...) {
                std::cout << "benchmark failed" << std::endl;
                run_benchmark_ = false;
            }
        }
#endif
        render_frame();
    }
    running_ = false;
}

std::filesystem::path Core::getPluginResourcesPath() const {
    if (currentPluginResourcesPath_.empty()) {
        throw currentPluginResourcesPathException_;
    }
    return currentPluginResourcesPath_;
}

bool Core::isKeyPressed(Key key) const {
    return glfwGetKey(window_, static_cast<int>(key)) == GLFW_PRESS;
}

bool Core::isMouseButtonPressed(MouseButton button) const {
    return glfwGetMouseButton(window_, static_cast<int>(button)) == GLFW_PRESS;
}

void Core::getMousePos(double& xpos, double& ypos) const {
    glfwGetCursorPos(window_, &xpos, &ypos);
    scaleWindowPosToFramebufferPos(xpos, ypos);
}

void Core::setWindowSize(int width, int height) const {
    glfwSetWindowSize(window_, width, height);
}

void Core::registerCamera(const std::shared_ptr<AbstractCamera>& camera) const {
    camera_ = camera;
}

void Core::removeCamera() const {
    camera_.reset();
}

#ifdef OGL4CORE2_ENABLE_PWROWG
void OGL4Core2::Core::Core::push_gaze_point(std::array<float, 2> const& gp) {
    recorded_gaze_points_.push_back(gp);
}

std::array<float, 2> OGL4Core2::Core::Core::pull_gaze_point() const {
    return current_gaze_point_;
}

bool OGL4Core2::Core::Core::record_gaze_point() const {
    return recording_paths_;
}

bool OGL4Core2::Core::Core::replay_gaze_point() const {
    return run_benchmark_ && replay_eyes_;
}
#endif

void Core::validateImGuiScale() {
    float xscale, yscale;
    glfwGetWindowContentScale(window_, &xscale, &yscale);

    // Different x and y scaling is not handled
    const float scale = (xscale + yscale) * 0.5f;

    if (contentScale_ != scale) {
        // Setup fonts
        auto& io = ImGui::GetIO();
        io.Fonts->Clear();

        // Default font settings with scaled size
        ImFontConfig fontConfig;
        fontConfig.SizePixels = 13.0f * scale;
        fontConfig.OversampleH = 1;
        fontConfig.OversampleV = 1;
        fontConfig.PixelSnapH = true;
        io.Fonts->AddFontDefault(&fontConfig);

        ImGui_ImplOpenGL3_DestroyFontsTexture();
        ImGui_ImplOpenGL3_CreateFontsTexture();

        // Setup style
        ImGui::GetStyle() = ImGuiStyle();
        ImGui::StyleColorsDark();
        ImGui::GetStyle().ScaleAllSizes(scale);

        contentScale_ = scale;
    }
}

void Core::draw() {
    validateImGuiScale();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(10.0, 10.0), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(300.0, 600.0), ImGuiCond_Once);

    ImGui::Begin(title);

    if (ImGui::CollapsingHeader("Plugins", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Combo("Plugin", &pluginSelectionIdx_, pluginNamesImGui_.data());
    }
#ifdef OGL4CORE2_ENABLE_PWROWG
    if (ImGui::CollapsingHeader("PwrOwg", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputInt2("Window Size", window_size_.data(),
            ImGuiInputTextFlags_::ImGuiInputTextFlags_EnterReturnsTrue);
        if (window_size_[0] != old_window_size_[0] || window_size_[1] != old_window_size_[1]) {
            setWindowSize(window_size_[0], window_size_[1]);
            old_window_size_ = window_size_;
        }
        ImGui::InputInt("Frame Cap [ms]", &frame_cap_ms_, 1, 100,
            ImGuiInputTextFlags_::ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        ImGui::Checkbox("Use Frame Cap", &use_frame_cap_);
        ImGui::SliderInt("Num Frames", &num_frames_, 1, 10000);
        char outPath[256];
        sprintf_s(outPath, "%s", output_file_.c_str());
        ImGui::InputText("Output Path", outPath, 256);
        output_file_ = std::string(outPath);
        if (ImGui::Button("Start Benchmark")) {
            auto const& sensors = reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensors;
            auto& bufs = reinterpret_cast<pwr_owg_config_t*>(pwr_owg_config_)->sensor_buffers;
            bufs.clear();
            bufs.resize(sensors.size());

            expected_samples_num_ = num_frames_ * 2 / sample_interval_; //< expecting 2ms per frame
            std::cout << "reserving for " << expected_samples_num_ << " samples" << std::endl;

            for (auto& b : bufs) {
                b.reserve(expected_samples_num_);
            }

            run_benchmark_ = true;
        }
        if (ImGui::CollapsingHeader("Automation", ImGuiTreeNodeFlags_DefaultOpen)) {
            char camPath[256];
            sprintf_s(camPath, "%s", camera_path_file_.c_str());
            ImGui::InputText("Cam Path", camPath, 256);
            camera_path_file_ = std::string(camPath);
            char eyePath[256];
            sprintf_s(eyePath, "%s", eye_path_file_.c_str());
            ImGui::InputText("Eye Path", eyePath, 256);
            eye_path_file_ = std::string(eyePath);
            if (ImGui::Checkbox("Replay", &replay_paths_)) {
                if (std::filesystem::exists(camera_path_file_)) {
                    auto const file_size = std::filesystem::file_size(camera_path_file_);
                    recorded_entries_.clear();
                    recorded_entries_.resize(file_size / sizeof(decltype(recorded_entries_)::value_type));
                    auto cam_file = std::ifstream(std::filesystem::path(camera_path_file_), std::ios::binary);
                    cam_file.read(reinterpret_cast<char*>(recorded_entries_.data()), file_size);
                    cam_file.close();
                    recorded_gaze_points_.clear();
                    if (std::filesystem::exists(eye_path_file_)) {
                        auto const eye_file_size = std::filesystem::file_size(eye_path_file_);
                        recorded_gaze_points_.resize(
                            eye_file_size / sizeof(decltype(recorded_gaze_points_)::value_type));
                        auto eye_file = std::ifstream(std::filesystem::path(eye_path_file_), std::ios::binary);
                        eye_file.read(reinterpret_cast<char*>(recorded_gaze_points_.data()), eye_file_size);
                        eye_file.close();
                        if (recorded_gaze_points_.size() != recorded_entries_.size()) {
                            recorded_gaze_points_.clear();
                            current_gaze_point_ = {0.5f, 0.5f};
                        }
                    }
                } else {
                    replay_paths_ = false;
                    current_gaze_point_ = {0.5f, 0.5f};
                }
            }
            ImGui::SameLine();
            ImGui::Checkbox("Replay Eyes", &replay_eyes_);
            if (ImGui::Button("Record")) {
                recording_paths_ = !recording_paths_;
                if (recording_paths_) {
                    recorded_entries_.clear();
                    recorded_entries_.reserve(100000);
                    recorded_gaze_points_.clear();
                    recorded_gaze_points_.reserve(100000);
                } else {
                    auto cam_file = std::ofstream(std::filesystem::path(camera_path_file_), std::ios::binary);
                    cam_file.write(reinterpret_cast<char const*>(recorded_entries_.data()),
                        recorded_entries_.size() * sizeof(decltype(recorded_entries_)::value_type));
                    cam_file.close();
                    auto eye_file = std::ofstream(std::filesystem::path(eye_path_file_), std::ios::binary);
                    eye_file.write(reinterpret_cast<char const*>(recorded_gaze_points_.data()),
                        recorded_gaze_points_.size() * sizeof(decltype(recorded_gaze_points_)::value_type));
                    eye_file.close();
                }
            }
            ImGui::SameLine();
            ImGui::Checkbox("Recording", &recording_paths_);
        }
    }
#endif
    if (currentPluginIdx_ != pluginSelectionIdx_) {
        currentPluginIdx_ = pluginSelectionIdx_;
        // Need to delete plugin first, so destructor of old plugin runs before constructor of new plugin.
        // Otherwise, this could mess up OpenGL states.
        currentPlugin_ = nullptr;

        // Init new plugin
        const auto& plugin = PluginRegister::get(currentPluginIdx_);

        // Get plugin resource dir. This is done here, that we can keep access to path const as plugins should only
        // get a const reference to core. But as having a resource dir is optional for plugins, we want to show an
        // exception only if a plugin tries to access the path. Therefore, catch the exception and cache it.
        try {
            currentPluginResourcesPath_ = FileUtil::findPluginResourcesPath(plugin->path());
        } catch (const std::exception& ex) {
            currentPluginResourcesPathException_ = ex;
            currentPluginResourcesPath_.clear();
        }

        currentPlugin_ = plugin->create(*this);
        // Plugin needs to know window size.
        currentPlugin_->resize(framebufferWidth_, framebufferHeight_);
    }

    glClear(GL_COLOR_BUFFER_BIT);

    if (currentPlugin_ != nullptr) {
        currentPlugin_->render();
    }

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Core::screenshot() {
    if (cfg_.screenshotFrames.empty() || cfg_.screenshotFrames.front() != frameNumber_) {
        return;
    }

    cfg_.screenshotFrames.erase(cfg_.screenshotFrames.begin());

    std::vector<unsigned char> image(framebufferWidth_ * framebufferHeight_ * 4);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, framebufferWidth_, framebufferHeight_, GL_RGBA, GL_UNSIGNED_BYTE, image.data());

    std::string filename = cfg_.screenshotFilename.empty() ? "screenshot" : cfg_.screenshotFilename;
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << frameNumber_;
    filename += "." + ss.str() + ".png";

    ImageUtil::savePngImage(filename, std::move(image), framebufferWidth_, framebufferHeight_);

    if (cfg_.autoQuit && cfg_.screenshotFrames.empty()) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

void Core::windowSizeEvent(int width, int height) {
    windowWidth_ = width;
    windowHeight_ = height;
}

void Core::framebufferSizeEvent(int width, int height) {
    // Save size for init of new plugin.
    framebufferWidth_ = width;
    framebufferHeight_ = height;
    if (currentPlugin_ != nullptr) {
        currentPlugin_->resize(width, height);
    }
}

void Core::keyEvent(int key, [[maybe_unused]] int scancode, int action, int mods) {
    mods = GLFWUtil::fixKeyboardMods(mods, key, action);
    if (!ImGui::GetIO().WantCaptureKeyboard && currentPlugin_ != nullptr) {
        currentPlugin_->keyboard(static_cast<Key>(key), static_cast<KeyAction>(action), Mods(mods));
    }
}

void Core::charEvent(unsigned int codepoint) {
    if (!ImGui::GetIO().WantTextInput && currentPlugin_ != nullptr) {
        currentPlugin_->charInput(codepoint);
    }
}

void Core::mouseButtonEvent(int button, int action, int mods) {
    auto b = static_cast<MouseButton>(button);
    auto a = static_cast<MouseButtonAction>(action);
    Mods m(mods);

    cameraControlMode_ = AbstractCamera::MouseControlMode::None;
    if (a == MouseButtonAction::Press && m.none()) {
        if (b == MouseButton::Left) {
            cameraControlMode_ = AbstractCamera::MouseControlMode::Left;
        } else if (b == MouseButton::Middle) {
            cameraControlMode_ = AbstractCamera::MouseControlMode::Middle;
        } else if (b == MouseButton::Right) {
            cameraControlMode_ = AbstractCamera::MouseControlMode::Right;
        }
    }

    if (!ImGui::GetIO().WantCaptureMouse && currentPlugin_ != nullptr) {
        currentPlugin_->mouseButton(b, a, m);
    }
}

void Core::mouseMoveEvent(double xpos, double ypos) {
    scaleWindowPosToFramebufferPos(xpos, ypos);

    if (!ImGui::GetIO().WantCaptureMouse && currentPlugin_ != nullptr) {
        // Check camera event in mouseButtonEvent for correct modifier state. Checking here just for current key status
        // with glfwGetKey will miss the state when the modifier key was pressed before the window gets the focus. The
        // reason for this is, that glfwGetKey only returns a cached state, while the modifiers parameter contains the
        // live status.
        if (cameraControlMode_ != AbstractCamera::MouseControlMode::None) {
            auto camera = camera_.lock();
            if (camera) {
                double oldX = 2.0 * mouseX_ / static_cast<double>(framebufferWidth_) - 1.0;
                double oldY = 1.0 - 2.0 * mouseY_ / static_cast<double>(framebufferHeight_);
                double newX = 2.0 * xpos / static_cast<double>(framebufferWidth_) - 1.0;
                double newY = 1.0 - 2.0 * ypos / static_cast<double>(framebufferHeight_);
                camera->mouseMoveControl(cameraControlMode_, oldX, oldY, newX, newY);
#ifdef OGL4CORE2_ENABLE_PWROWG
                if (recording_paths_) {
                    current_entry_.mode = cameraControlMode_;
                    current_entry_.oldX = oldX;
                    current_entry_.oldY = oldY;
                    current_entry_.newX = newX;
                    current_entry_.newY = newY;
                }
#endif
            }
        }

        currentPlugin_->mouseMove(xpos, ypos);
    }
    mouseX_ = xpos;
    mouseY_ = ypos;
}

void Core::mouseScrollEvent(double xoffset, double yoffset) {
    if (!ImGui::GetIO().WantCaptureMouse && currentPlugin_ != nullptr) {
        if (!GLFWUtil::anyModKeyPressed(window_)) {
            auto camera = camera_.lock();
            if (camera) {
                camera->mouseScrollControl(xoffset, yoffset);
#ifdef OGL4CORE2_ENABLE_PWROWG
                if (recording_paths_) {
                    current_entry_.xoffset = xoffset;
                    current_entry_.yoffset = yoffset;
                }
#endif
            }
        }
        currentPlugin_->mouseScroll(xoffset, yoffset);
    }
}

int Core::glfwReferenceCounter_ = 0;

void Core::initGLFW() {
    if (Core::glfwReferenceCounter_ <= 0) {
        glfwSetErrorCallback([](int error_code, const char* description) {
            std::cerr << "GLFW Error (" << error_code << "): " << description << std::endl;
        });
        if (!glfwInit()) {
            throw std::runtime_error("GLFW init failed!");
        }
    }
    Core::glfwReferenceCounter_++;
}

void Core::terminateGLFW() {
    Core::glfwReferenceCounter_--;
    if (Core::glfwReferenceCounter_ <= 0) {
        glfwTerminate();
        Core::glfwReferenceCounter_ = 0;
    }
}

void Core::scaleWindowPosToFramebufferPos(double& xpos, double& ypos) const {
    xpos *= static_cast<double>(framebufferWidth_) / static_cast<double>(windowWidth_);
    ypos *= static_cast<double>(framebufferHeight_) / static_cast<double>(windowHeight_);
}
