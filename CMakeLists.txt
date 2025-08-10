# ──────────────────────────────────────────────────────────────
#  root   CMakeLists.txt   (…/stag/CMakeLists.txt)
# ──────────────────────────────────────────────────────────────
cmake_minimum_required(VERSION 3.16)
project(Stag LANGUAGES C CXX)

# ----------- 0. Defaults -------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# MSVC niceties
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Build everything PIC so STATIC code can live in a shared object
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------- 1. Compiler flags ------------------------------------------
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX)
add_compile_options(
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)
endif(CMAKE_COMPILER_IS_GNUCXX)

# ----------- 2. Dependencies --------------------------------------------
find_package(OpenCV 4.12 REQUIRED COMPONENTS world)
# world supplies all headers; no need to list imgproc/imgcodecs separately

# ----------- 3. staglib (STATIC) ----------------------------------------
file(GLOB SRC_CORE "src/*.c*")
file(GLOB SRC_ED   "src/ED/*.c*")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(staglib STATIC
    ${SRC_CORE}
    ${SRC_ED}
    src/Stag.cpp     # explicit if not matched by GLOB
)
target_include_directories(staglib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ED>
)
target_link_libraries(staglib PUBLIC
    opencv_world            # PUBLIC so dependents inherit include dirs
)

# ----------- 4. (optional) sample executable -----------------------------
add_executable(stag_example main.cpp)
target_link_libraries(stag_example PRIVATE staglib)

# Keep Windows output next to the .exe for convenience
if(MSVC)
    set_target_properties(stag_example PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY          ${PROJECT_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY_DEBUG    ${PROJECT_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${PROJECT_BINARY_DIR}
    )
endif()

# ----------- 5. Wrapper --------------------------------------------------
# Build stagdll in a subdirectory; its own CMakeLists handles linkage
add_subdirectory(stagdll)       # ← path relative to this root


# ──────────────────────────────────────────────────────────────
