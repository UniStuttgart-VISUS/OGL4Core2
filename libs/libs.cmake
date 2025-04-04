# Dependencies

# Setup FetchContent
include(FetchContent)
mark_as_advanced(FORCE
  FETCHCONTENT_BASE_DIR
  FETCHCONTENT_FULLY_DISCONNECTED
  FETCHCONTENT_QUIET
  FETCHCONTENT_UPDATES_DISCONNECTED)

# cxxopts
FetchContent_Declare(cxxopts
  URL "https://github.com/jarro2783/cxxopts/archive/v3.2.0.tar.gz"
  URL_HASH SHA256=9f43fa972532e5df6c5fd5ad0f5bac606cdec541ccaf1732463d8070bbb7f03b)
FetchContent_GetProperties(cxxopts)
if (NOT cxxopts_POPULATED)
  message(STATUS "Fetch cxxopts ...")
  FetchContent_Populate(cxxopts)
  add_subdirectory(${cxxopts_SOURCE_DIR} ${cxxopts_BINARY_DIR} EXCLUDE_FROM_ALL)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_CXXOPTS
    FETCHCONTENT_UPDATES_DISCONNECTED_CXXOPTS
    CXXOPTS_BUILD_EXAMPLES
    CXXOPTS_BUILD_TESTS
    CXXOPTS_ENABLE_INSTALL
    CXXOPTS_ENABLE_WARNINGS
    CXXOPTS_USE_UNICODE_HELP)
endif ()

# GLFW
FetchContent_Declare(glfw
  URL "https://github.com/glfw/glfw/archive/3.4.tar.gz"
  URL_HASH SHA256=c038d34200234d071fae9345bc455e4a8f2f544ab60150765d7704e08f3dac01)
FetchContent_GetProperties(glfw)
if (NOT glfw_POPULATED)
  message(STATUS "Fetch glfw ...")
  FetchContent_Populate(glfw)
  option(GLFW_BUILD_DOCS "" OFF)
  option(GLFW_BUILD_EXAMPLES "" OFF)
  option(GLFW_BUILD_TESTS "" OFF)
  option(GLFW_INSTALL "" OFF)
  set(GLFW_LIBRARY_TYPE "STATIC" CACHE STRING "")
  if (WIN32)
    option(GLFW_USE_HYBRID_HPG "" ON)
    option(USE_MSVC_RUNTIME_LIBRARY_DLL "" OFF)
  elseif (UNIX AND NOT APPLE)
    # Detect X11 or Wayland
    if ("$ENV{XDG_SESSION_TYPE}" STREQUAL "wayland")
      option(GLFW_BUILD_X11 "" OFF)
    else()
      option(GLFW_BUILD_WAYLAND "" OFF)
    endif ()
  endif ()
  add_subdirectory(${glfw_SOURCE_DIR} ${glfw_BINARY_DIR} EXCLUDE_FROM_ALL)
  set_target_properties(glfw PROPERTIES FOLDER libs)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_GLFW
    FETCHCONTENT_UPDATES_DISCONNECTED_GLFW
    BUILD_SHARED_LIBS
    GLFW_BUILD_DOCS
    GLFW_BUILD_EXAMPLES
    GLFW_BUILD_TESTS
    GLFW_BUILD_WAYLAND
    GLFW_BUILD_WIN32
    GLFW_BUILD_X11
    GLFW_INSTALL
    GLFW_LIBRARY_TYPE
    GLFW_USE_HYBRID_HPG
    USE_MSVC_RUNTIME_LIBRARY_DLL
    X11_xcb_icccm_INCLUDE_PATH
    X11_xcb_icccm_LIB)
endif ()

# glad2
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/glad/ EXCLUDE_FROM_ALL)
set_target_properties(glad PROPERTIES FOLDER libs)

# glm
FetchContent_Declare(glm
  URL "https://github.com/g-truc/glm/archive/1.0.1.tar.gz"
  URL_HASH SHA256=9f3174561fd26904b23f0db5e560971cbf9b3cbda0b280f04d5c379d03bf234c)
FetchContent_GetProperties(glm)
if (NOT glm_POPULATED)
  message(STATUS "Fetch glm ...")
  FetchContent_Populate(glm)
  add_subdirectory(${glm_SOURCE_DIR} ${glm_BINARY_DIR} EXCLUDE_FROM_ALL)
  target_compile_definitions(glm PUBLIC GLM_ENABLE_EXPERIMENTAL)
  set_target_properties(glm PROPERTIES FOLDER libs)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_GLM
    FETCHCONTENT_UPDATES_DISCONNECTED_GLM
    GLM_BUILD_INSTALL
    GLM_BUILD_LIBRARY
    GLM_BUILD_TESTS
    GLM_DISABLE_AUTO_DETECTION
    GLM_ENABLE_CXX_11
    GLM_ENABLE_CXX_14
    GLM_ENABLE_CXX_17
    GLM_ENABLE_CXX_20
    GLM_ENABLE_CXX_98
    GLM_ENABLE_FAST_MATH
    GLM_ENABLE_LANG_EXTENSIONS
    GLM_ENABLE_SIMD_AVX
    GLM_ENABLE_SIMD_AVX2
    GLM_ENABLE_SIMD_SSE2
    GLM_ENABLE_SIMD_SSE3
    GLM_ENABLE_SIMD_SSE4_1
    GLM_ENABLE_SIMD_SSE4_2
    GLM_ENABLE_SIMD_SSSE3
    GLM_FORCE_PURE)
endif ()

# glowl
FetchContent_Declare(glowl
  URL "https://github.com/invor/glowl/archive/e075724a649bd1d57e464d9432556fb69be22699.tar.gz"
  URL_HASH SHA256=aa4d556d4942105d5159a098cceb2845ac173fb80bda240de164f11e88d08f05)
FetchContent_GetProperties(glowl)
if (NOT glowl_POPULATED)
  message(STATUS "Fetch glowl ...")
  FetchContent_Populate(glowl)
  set(GLOWL_OPENGL_INCLUDE "GLAD2" CACHE STRING "" FORCE)
  option(GLOWL_USE_ARB_BINDLESS_TEXTURE "" OFF)
  add_subdirectory(${glowl_SOURCE_DIR} ${glowl_BINARY_DIR} EXCLUDE_FROM_ALL)
  # Mark include dirs as 'system' to disable warnings.
  get_target_property(include_dirs glowl INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(glowl PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${include_dirs}")
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_GLOWL
    FETCHCONTENT_UPDATES_DISCONNECTED_GLOWL
    GLOWL_OPENGL_INCLUDE
    GLOWL_USE_ARB_BINDLESS_TEXTURE
    GLOWL_USE_GLM
    GLOWL_USE_NV_MESH_SHADER)
endif ()

# imgui
FetchContent_Declare(imgui
  URL "https://github.com/ocornut/imgui/archive/v1.90.4.tar.gz"
  URL_HASH SHA256=5d9dc738af74efa357f2a9fc39fe4a28d29ef1dfc725dd2977ccf3f3194e996e)
FetchContent_GetProperties(imgui)
if (NOT imgui_POPULATED)
  message(STATUS "Fetch imgui ...")
  FetchContent_Populate(imgui)
  file(COPY ${CMAKE_SOURCE_DIR}/libs/imgui/CMakeLists.txt DESTINATION ${imgui_SOURCE_DIR})
  add_subdirectory(${imgui_SOURCE_DIR} ${imgui_BINARY_DIR} EXCLUDE_FROM_ALL)
  target_link_libraries(imgui PRIVATE glfw)
  set_target_properties(imgui PROPERTIES FOLDER libs)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_IMGUI
    FETCHCONTENT_UPDATES_DISCONNECTED_IMGUI)
endif ()

# imGuIZMO.quat
FetchContent_Declare(imguizmo
  URL "https://github.com/BrutPitt/imGuIZMO.quat/archive/v3.0.tar.gz"
  URL_HASH SHA256=b088ed085c51615ad407a32dbaec6e0b99ab5d894f4e79937bb13f85a30e6a1f)
FetchContent_GetProperties(imguizmo)
if (NOT imguizmo_POPULATED)
  message(STATUS "Fetch imguizmo ...")
  FetchContent_Populate(imguizmo)
  file(COPY ${CMAKE_SOURCE_DIR}/libs/imguizmo/CMakeLists.txt DESTINATION ${imguizmo_SOURCE_DIR})
  add_subdirectory(${imguizmo_SOURCE_DIR} ${imguizmo_BINARY_DIR} EXCLUDE_FROM_ALL)
  target_link_libraries(imguizmo PRIVATE imgui glm)
  get_target_property(include_dirs imguizmo INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(imguizmo PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${include_dirs}"
    FOLDER libs)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_IMGUIZMO
    FETCHCONTENT_UPDATES_DISCONNECTED_IMGUIZMO)
endif ()

# LodePNG
FetchContent_Declare(lodepng
  URL "https://github.com/lvandeve/lodepng/archive/d398e0f10d152a5d17fa30463474dc9f56523f9c.tar.gz"
  URL_HASH SHA256=52cc82678d9fd6750d7962fd56bd3d987b302b7ee554c1ac42f5abada7afd6bc)
FetchContent_GetProperties(lodepng)
if (NOT lodepng_POPULATED)
  message(STATUS "Fetch lodepng ...")
  FetchContent_Populate(lodepng)
  file(COPY ${CMAKE_SOURCE_DIR}/libs/lodepng/CMakeLists.txt DESTINATION ${lodepng_SOURCE_DIR})
  add_subdirectory(${lodepng_SOURCE_DIR} ${lodepng_BINARY_DIR} EXCLUDE_FROM_ALL)
  set_target_properties(lodepng PROPERTIES FOLDER libs)
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_LODEPNG
    FETCHCONTENT_UPDATES_DISCONNECTED_LODEPNG)
endif ()

# datraw
FetchContent_Declare(datraw
  URL "https://github.com/UniStuttgart-VISUS/datraw/archive/v1.0.9.tar.gz"
  URL_HASH SHA256=2cfb392bbd9bc8fae270ee08ae874c1ec22307c07d16c6a5515b5eaef42b0139)
FetchContent_GetProperties(datraw)
if (NOT datraw_POPULATED)
  message(STATUS "Fetch datraw ...")
  FetchContent_Populate(datraw)
  file(COPY ${CMAKE_SOURCE_DIR}/libs/datraw/CMakeLists.txt DESTINATION ${datraw_SOURCE_DIR})
  add_subdirectory(${datraw_SOURCE_DIR} ${datraw_BINARY_DIR} EXCLUDE_FROM_ALL)
  get_target_property(include_dirs datraw INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(datraw PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${include_dirs}")
  mark_as_advanced(FORCE
    FETCHCONTENT_SOURCE_DIR_DATRAW
    FETCHCONTENT_UPDATES_DISCONNECTED_DATRAW)
endif ()

# boost stacktrace
if (OGL4CORE2_ENABLE_STACKTRACE)
  FetchContent_Declare(stacktrace
    URL "https://github.com/UniStuttgart-VISUS/boost-stacktrace-zip/releases/download/boost-1.80.0/boost-stacktrace.zip"
    URL_HASH SHA256=394c963eaef339d32d08162a4253a96bbc5197d768133877e29fa6ad5bdd7570)
  FetchContent_GetProperties(stacktrace)
  if (NOT stacktrace_POPULATED)
    message(STATUS "Fetch stacktrace ...")
    FetchContent_Populate(stacktrace)
    add_subdirectory(${stacktrace_SOURCE_DIR} ${stacktrace_BINARY_DIR} EXCLUDE_FROM_ALL)
    if (WIN32)
      set_target_properties(boost_stacktrace_windbg PROPERTIES FOLDER libs)
    endif ()
    mark_as_advanced(FORCE
      FETCHCONTENT_SOURCE_DIR_STACKTRACE
      FETCHCONTENT_UPDATES_DISCONNECTED_STACKTRACE
      BOOST_REGEX_STANDALONE
      BOOST_STACKTRACE_ENABLE_ADDR2LINE
      BOOST_STACKTRACE_ENABLE_BACKTRACE
      BOOST_STACKTRACE_ENABLE_BASIC
      BOOST_STACKTRACE_ENABLE_NOOP
      BOOST_STACKTRACE_ENABLE_WINDBG
      BOOST_STACKTRACE_ENABLE_WINDBG_CACHED
      BOOST_THREAD_THREADAPI
      ICU_INCLUDE_DIR)
  endif ()
endif ()
