cmake_minimum_required(VERSION 3.24)
project(opennuts_gmk
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "OpenNuts Geometric Modeling Kernel (GMK)"
)

# ---------------------------------------------------------------------------
# Standard selection. We target C++23 but fall back gracefully on toolchains
# that only advertise older standards. Library code itself avoids bleeding
# edge std features so it builds on C++20 as well.
# ---------------------------------------------------------------------------
function(_gmk_pick_cxx_std out_var)
    set(_candidates 23 20)
    foreach(_n IN LISTS _candidates)
        if("cxx_std_${_n}" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
            set(${out_var} ${_n} PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${out_var} 20 PARENT_SCOPE)
endfunction()
_gmk_pick_cxx_std(GMK_CXX_STANDARD)
message(STATUS "gmk: building with C++${GMK_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD ${GMK_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# ---------------------------------------------------------------------------
# Options.
# ---------------------------------------------------------------------------
option(GMK_BUILD_CLI    "Build native CLI (test/lsp/geom-server)" ON)
option(GMK_BUILD_WASM   "Build WASM bindings (emscripten)"        OFF)
option(GMK_BUILD_TESTS  "Build unit tests"                        ON)

if(EMSCRIPTEN)
    set(GMK_BUILD_WASM  ON  CACHE BOOL "" FORCE)
    set(GMK_BUILD_CLI   OFF CACHE BOOL "" FORCE)
    # tests use native exec; not meaningful under emscripten
    set(GMK_BUILD_TESTS OFF CACHE BOOL "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Strict warnings, no exceptions on hot paths, but allow them for the LSP /
# JSON layer where the user-facing recovery is easier with exceptions.
# ---------------------------------------------------------------------------
function(gmk_apply_flags tgt)
    if(MSVC)
        target_compile_options(${tgt} PRIVATE /W4 /permissive- /Zc:__cplusplus)
    else()
        target_compile_options(${tgt} PRIVATE
            -Wall -Wextra -Wpedantic
            -Wshadow -Wconversion -Wsign-conversion -Wnon-virtual-dtor
            -Wold-style-cast -Wcast-align -Woverloaded-virtual
            -Wno-unused-parameter
        )
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Core library: the geometric modeling kernel itself. Header + impl-only,
# zero external dependencies, only the C++ standard library.
# ---------------------------------------------------------------------------
set(GMK_CORE_SOURCES
    src/math/transform.cpp
    src/geom/nurbs_curve.cpp
    src/geom/analytic_curves.cpp
    src/geom/nurbs_surface.cpp
    src/geom/analytic_surfaces.cpp
    src/brep/body.cpp
    src/brep/primitives.cpp
    src/brep/validate.cpp
    src/ops/boolean.cpp
    src/tess/tessellator.cpp
    src/lang/lexer.cpp
    src/lang/parser.cpp
    src/lang/eval.cpp
    src/server/json.cpp
    src/server/jsonrpc.cpp
    src/server/lsp_server.cpp
    src/server/geom_server.cpp
)
add_library(gmk STATIC ${GMK_CORE_SOURCES})
target_include_directories(gmk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(gmk PRIVATE src)
gmk_apply_flags(gmk)

# ---------------------------------------------------------------------------
# Native CLI / dual server (LSP + geometry).
# ---------------------------------------------------------------------------
if(GMK_BUILD_CLI)
    add_executable(opennuts src/cli/main.cpp)
    target_link_libraries(opennuts PRIVATE gmk)
    gmk_apply_flags(opennuts)
endif()

# ---------------------------------------------------------------------------
# WASM build.
# ---------------------------------------------------------------------------
if(GMK_BUILD_WASM)
    if(NOT EMSCRIPTEN)
        message(FATAL_ERROR
            "GMK_BUILD_WASM=ON but compiler is not emscripten. "
            "Re-configure with: emcmake cmake -S . -B build-wasm")
    endif()
    add_executable(gmk_wasm wasm/entry.cpp)
    target_link_libraries(gmk_wasm PRIVATE gmk)
    gmk_apply_flags(gmk_wasm)
    target_link_options(gmk_wasm PRIVATE
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createGmkModule"
        "-sENVIRONMENT=web,worker,node"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sEXPORT_ES6=0"
        "-sWASM=1"
        "-sNO_EXIT_RUNTIME=1"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','UTF8ToString','stringToUTF8','lengthBytesUTF8']"
        "-sEXPORTED_FUNCTIONS=['_malloc','_free','_gmk_version','_gmk_session_new','_gmk_session_free','_gmk_session_dispatch']"
    )
    set_target_properties(gmk_wasm PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        SUFFIX ".js"
    )
endif()

# ---------------------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------------------
if(GMK_BUILD_TESTS)
    enable_testing()
    set(_test_srcs
        tests/test_units.cpp
        tests/test_math.cpp
        tests/test_nurbs.cpp
        tests/test_brep.cpp
        tests/test_lang.cpp
        tests/test_json.cpp
    )
    add_executable(gmk_tests tests/test_main.cpp ${_test_srcs})
    target_link_libraries(gmk_tests PRIVATE gmk)
    gmk_apply_flags(gmk_tests)
    add_test(NAME gmk_tests COMMAND gmk_tests)
endif()

# ---------------------------------------------------------------------------
# Install layout (only sensible for native builds).
# ---------------------------------------------------------------------------
if(GMK_BUILD_CLI)
    install(TARGETS opennuts RUNTIME DESTINATION bin)
endif()
install(DIRECTORY include/gmk DESTINATION include)
