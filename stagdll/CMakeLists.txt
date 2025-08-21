cmake_minimum_required(VERSION 3.20)
project(stagdll LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ────────────────────── 1. Dependencies ──────────────────────
# Monolithic build of OpenCV (libopencv_world.so / opencv_world4120.dll)
find_package(OpenCV 4.12 REQUIRED COMPONENTS world)

# ────────────────────── 2. Library target ────────────────────
add_library(stagdll SHARED stag_wrapper.cpp)

# Where your own headers live (adjust if you have an 'include' dir)
target_include_directories(stagdll
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..        # pch.h, Stag.h, …
)

# Optional PCH for MSVC only
if (MSVC)
    target_precompile_headers(stagdll PRIVATE pch.h)
endif()

# Link against the imported OpenCV target
target_link_libraries(stagdll
    PRIVATE
	staglib
		opencv_world                  # pulls in include dirs & linker flags
)

# ────────────────────── 3. Niceties ──────────────────────────

set_target_properties(stagdll PROPERTIES
    BUILD_RPATH   "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)

set_target_properties(stagdll PROPERTIES
    OUTPUT_NAME "stagdll"            # stagdll.dll / libstagdll.so
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)
