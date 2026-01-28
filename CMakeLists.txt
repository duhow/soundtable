cmake_minimum_required(VERSION 3.20)

project(soundtable VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Project-wide options
option(SOUNDTABLE_ENABLE_TRACKER "Build the tracker service" ON)
option(SOUNDTABLE_BUILD_TESTS "Build C++ tests" ON)

# External reacTIVision/libfidtrack sources that generate many
# conversion/sign-conversion/float-conversion warnings. Treat them as
# third-party code and relax those warnings only for these files,
# while keeping a high warning level on our own code.
set(REACTIVISION_SOURCES
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/segment.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/fidtrackX.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/topologysearch.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/treeidmap.cpp
)

if (NOT MSVC)
    set_source_files_properties(${REACTIVISION_SOURCES}
        PROPERTIES
            COMPILE_OPTIONS "-Wno-conversion;-Wno-sign-conversion;-Wno-float-conversion;-Wno-unused-parameter"
    )
endif()

# Optional CPU-specific optimization flags (non-MSVC toolchains).
option(SOUNDTABLE_ENABLE_CPU_OPTIMIZATIONS "Enable CPU-specific optimization flags (e.g. -march/-mtune)" ON)

if (SOUNDTABLE_ENABLE_CPU_OPTIMIZATIONS AND NOT MSVC)
    include(CheckCXXCompilerFlag)

    set(SOUNDTABLE_CPU_OPT_FLAGS "")

    check_cxx_compiler_flag("-march=native" SOUNDTABLE_HAS_MARCH_NATIVE)
    if (SOUNDTABLE_HAS_MARCH_NATIVE)
        list(APPEND SOUNDTABLE_CPU_OPT_FLAGS "-march=native")
    endif()

    check_cxx_compiler_flag("-mtune=native" SOUNDTABLE_HAS_MTUNE_NATIVE)
    if (SOUNDTABLE_HAS_MTUNE_NATIVE)
        list(APPEND SOUNDTABLE_CPU_OPT_FLAGS "-mtune=native")
    endif()

    foreach(flag IN LISTS SOUNDTABLE_CPU_OPT_FLAGS)
        add_compile_options(
            "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${flag}>"
        )
    endforeach()
endif()

# Global floating-point and frame-pointer flags for GCC/Clang.
if (NOT MSVC)
    # Use -ffast-math in Release/RelWithDebInfo to allow more
    # aggressive floating-point optimizations in performance builds.
    add_compile_options(
        "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-ffast-math>"
    )

    # Keep frame pointers in Debug builds for better profiling and
    # easier stack traces.
    add_compile_options(
        "$<$<CONFIG:Debug>:-fno-omit-frame-pointer>"
    )
endif()

include(CTest)

# Path to JUCE as a submodule (expected at JUCE/)
set(JUCE_DIR "${CMAKE_SOURCE_DIR}/JUCE" CACHE PATH "Path to JUCE source directory")

if (NOT EXISTS "${JUCE_DIR}/CMakeLists.txt")
    message(WARNING "JUCE not found in ${JUCE_DIR}. Add JUCE as a submodule: git submodule add https://github.com/juce-framework/JUCE.git JUCE")
endif()

if (NOT EXISTS "${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/fidtrackX.c")
    message(WARNING "reacTIVision not found in ${CMAKE_SOURCE_DIR}/reacTIVision. Run: git clone https://github.com/mkalten/reacTIVision")
endif()

add_subdirectory(core)

if(SOUNDTABLE_ENABLE_TRACKER)
    add_subdirectory(tracker)
endif()

if(SOUNDTABLE_BUILD_TESTS)
    add_subdirectory(tests)
endif()