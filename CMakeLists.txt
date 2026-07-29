cmake_minimum_required(VERSION 3.24)

project(
    OrbitLab
    VERSION 1.0.0
    DESCRIPTION "Interactive real-time N-body gravity simulator"
    LANGUAGES CXX
)

option(ORBITLAB_BUILD_APP "Build the SDL/ImGui desktop application" ON)
option(ORBITLAB_BUILD_CLI "Build the headless command-line application" ON)
option(ORBITLAB_BUILD_TESTS "Build unit tests" ON)
option(ORBITLAB_BUILD_BENCHMARK "Build the performance harness" ON)
option(ORBITLAB_BUILD_FUZZER "Build the libFuzzer JSON parser target" OFF)
option(ORBITLAB_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ORBITLAB_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(ORBITLAB_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(ORBITLAB_ENABLE_COVERAGE "Enable source coverage instrumentation" OFF)
option(ORBITLAB_ENABLE_CLANG_TIDY "Run clang-tidy during compilation" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(ORBITLAB_ENABLE_ASAN AND ORBITLAB_ENABLE_TSAN)
    message(FATAL_ERROR "AddressSanitizer and ThreadSanitizer cannot be enabled together")
endif()

if(ORBITLAB_ENABLE_CLANG_TIDY)
    find_program(ORBITLAB_CLANG_TIDY_EXECUTABLE NAMES clang-tidy REQUIRED)
    set(ORBITLAB_CLANG_TIDY_COMMAND
        "${ORBITLAB_CLANG_TIDY_EXECUTABLE};--config-file=${CMAKE_SOURCE_DIR}/.clang-tidy")
endif()

add_library(orbitlab_instrumentation INTERFACE)
if(ORBITLAB_ENABLE_ASAN OR ORBITLAB_ENABLE_UBSAN OR ORBITLAB_ENABLE_TSAN)
    if(MSVC)
        if(ORBITLAB_ENABLE_ASAN AND NOT ORBITLAB_ENABLE_UBSAN)
            target_compile_options(orbitlab_instrumentation INTERFACE /fsanitize=address)
        else()
            message(FATAL_ERROR "Requested sanitizer combination is not supported by MSVC")
        endif()
    else()
        set(ORBITLAB_SANITIZERS "")
        if(ORBITLAB_ENABLE_ASAN)
            list(APPEND ORBITLAB_SANITIZERS address)
        endif()
        if(ORBITLAB_ENABLE_UBSAN)
            list(APPEND ORBITLAB_SANITIZERS undefined)
        endif()
        if(ORBITLAB_ENABLE_TSAN)
            list(APPEND ORBITLAB_SANITIZERS thread)
        endif()
        list(JOIN ORBITLAB_SANITIZERS "," ORBITLAB_SANITIZER_FLAGS)
        target_compile_options(orbitlab_instrumentation INTERFACE
            -fsanitize=${ORBITLAB_SANITIZER_FLAGS}
            -fno-omit-frame-pointer
        )
        target_link_options(orbitlab_instrumentation INTERFACE
            -fsanitize=${ORBITLAB_SANITIZER_FLAGS}
        )
    endif()
endif()
if(ORBITLAB_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(orbitlab_instrumentation INTERFACE -O0 -g --coverage)
        target_link_options(orbitlab_instrumentation INTERFACE --coverage)
    else()
        message(FATAL_ERROR "Coverage instrumentation requires GCC or Clang")
    endif()
endif()

include(FetchContent)
find_package(Threads REQUIRED)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
)
FetchContent_MakeAvailable(nlohmann_json)

add_library(orbitlab_core
    src/core/AdaptiveExperiment.cpp
    src/core/AdaptiveFidelity.cpp
    src/core/BarnesHutGravitySolver.cpp
    src/core/DirectGravitySolver.cpp
    src/core/Persistence.cpp
    src/core/Presets.cpp
    src/core/Replay.cpp
    src/core/Simulation.cpp
    src/core/SoAGravitySolver.cpp
    src/core/Validation.cpp
    src/core/WorkerPool.cpp
)
target_include_directories(orbitlab_core PUBLIC include)
target_link_libraries(orbitlab_core
    PUBLIC nlohmann_json::nlohmann_json Threads::Threads
    PRIVATE orbitlab_instrumentation
)

function(orbitlab_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    endif()
endfunction()
function(orbitlab_enable_analysis target)
    if(ORBITLAB_ENABLE_CLANG_TIDY)
        set_property(TARGET ${target} PROPERTY
            CXX_CLANG_TIDY "${ORBITLAB_CLANG_TIDY_COMMAND}")
    endif()
endfunction()
orbitlab_enable_warnings(orbitlab_core)
orbitlab_enable_analysis(orbitlab_core)

if(ORBITLAB_BUILD_CLI)
    add_executable(orbitlab_cli src/cli/main.cpp)
    target_link_libraries(orbitlab_cli PRIVATE orbitlab_core orbitlab_instrumentation)
    orbitlab_enable_warnings(orbitlab_cli)
    orbitlab_enable_analysis(orbitlab_cli)
endif()

if(ORBITLAB_BUILD_APP)
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        SDL
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.4.12
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.9
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(SDL imgui)

    add_library(orbitlab_imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp
    )
    target_include_directories(orbitlab_imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(orbitlab_imgui PUBLIC SDL3::SDL3)

    add_executable(OrbitLab
        src/app/Application.cpp
        src/app/Camera.cpp
        src/app/GpuGravitySolver.cpp
        src/app/GpuPlatform.cpp
        src/app/History.cpp
        src/app/Renderer.cpp
        src/app/UserInterface.cpp
        src/main.cpp
    )
    target_include_directories(OrbitLab PRIVATE src)
    target_link_libraries(OrbitLab PRIVATE
        orbitlab_core orbitlab_imgui SDL3::SDL3 orbitlab_instrumentation)
    orbitlab_enable_warnings(OrbitLab)
    orbitlab_enable_analysis(OrbitLab)
    add_custom_command(
        TARGET OrbitLab POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:OrbitLab>/assets
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${imgui_SOURCE_DIR}/misc/fonts/Karla-Regular.ttf
                $<TARGET_FILE_DIR:OrbitLab>/assets/Karla-Regular.ttf
        VERBATIM
    )
endif()

if(ORBITLAB_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.15.3
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(orbitlab_tests
        src/app/Camera.cpp
        src/app/History.cpp
        tests/AdaptiveFidelityTests.cpp
        tests/BarnesHutTests.cpp
        tests/CameraTests.cpp
        tests/CollisionTests.cpp
        tests/IntegrationTests.cpp
        tests/DiagnosticsTests.cpp
        tests/HistoryTests.cpp
        tests/PersistenceTests.cpp
        tests/ReplayTests.cpp
        tests/SolverTests.cpp
        tests/Vec2Tests.cpp
        tests/Vec3Tests.cpp
        tests/ValidationTests.cpp
        tests/WorkerPoolTests.cpp
    )
    target_include_directories(orbitlab_tests PRIVATE src)
    target_link_libraries(orbitlab_tests PRIVATE
        orbitlab_core Catch2::Catch2WithMain orbitlab_instrumentation)
    orbitlab_enable_warnings(orbitlab_tests)
    orbitlab_enable_analysis(orbitlab_tests)
    include(Catch)
    catch_discover_tests(orbitlab_tests)
endif()

if(ORBITLAB_BUILD_BENCHMARK)
    add_executable(orbitlab_benchmark benchmarks/SimulationBenchmark.cpp)
    target_link_libraries(orbitlab_benchmark PRIVATE orbitlab_core orbitlab_instrumentation)
    orbitlab_enable_warnings(orbitlab_benchmark)
    orbitlab_enable_analysis(orbitlab_benchmark)
endif()

if(ORBITLAB_BUILD_FUZZER)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "The OrbitLab fuzz target requires Clang and libFuzzer")
    endif()
    add_executable(orbitlab_json_fuzzer fuzz/JsonPersistenceFuzzer.cpp)
    target_link_libraries(orbitlab_json_fuzzer PRIVATE orbitlab_core orbitlab_instrumentation)
    target_compile_options(orbitlab_json_fuzzer PRIVATE -fsanitize=fuzzer,address)
    target_link_options(orbitlab_json_fuzzer PRIVATE -fsanitize=fuzzer,address)
    orbitlab_enable_warnings(orbitlab_json_fuzzer)
    orbitlab_enable_analysis(orbitlab_json_fuzzer)
endif()
