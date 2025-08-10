# ──────────────────────────────────────────────────────────────
#  stagdll/CMakeLists.txt   (…/stag/stagdll/CMakeLists.txt)
# ──────────────────────────────────────────────────────────────
cmake_minimum_required(VERSION 3.20)
project(stagdll LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------- 1. Dependencies --------------------------------------------
find_package(OpenCV 4.12 REQUIRED COMPONENTS world)

# ----------- 2. Library target ------------------------------------------
add_library(stagdll SHARED stag_wrapper.cpp)

target_include_directories(stagdll PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..   # access pch.h, Stag.h, etc.
)

# MSVC pre-compiled header (optional)
if(MSVC)
    target_precompile_headers(stagdll PRIVATE pch.h)
endif()

# STATIC staglib comes from parent directory; opencv_world is shared
target_link_libraries(stagdll
    PRIVATE
        staglib         # static → object code embedded
        opencv_world
)

# ----------- 3. RPATH / SONAME niceties ---------------------------------
set_target_properties(stagdll PROPERTIES
    OUTPUT_NAME    "stagdll"        # stagdll.dll / libstagdll.so
    BUILD_RPATH    "$ORIGIN"
    INSTALL_RPATH  "$ORIGIN"
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

# ──────────────────────────────────────────────────────────────
