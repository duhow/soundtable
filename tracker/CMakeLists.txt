cmake_minimum_required(VERSION 3.20)

project(soundtable-tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED core imgproc videoio highgui)

add_executable(soundtable-tracker
    src/main.cpp
    src/OscSender.cpp
    src/TuioNegotiation.cpp
    src/TrackerTypes.h
    src/TrackerState.cpp
    src/TrackerEngine.cpp
    ${REACTIVISION_SOURCES}
)

# Place the executable in the top-level build directory so that
# Earthfile can export it from build/soundtable-tracker
set_target_properties(soundtable-tracker PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "\$ORIGIN/lib"
)

if (MSVC)
    target_compile_options(soundtable-tracker PRIVATE /W4 /permissive-)
else()
    target_compile_options(soundtable-tracker PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

# Optional AVX/AVX2 optimizations specifically for soundtable-tracker.
# These flags are disabled by default to avoid generating binaries that
# cannot run on older CPUs. When enabled, they are only applied to
# optimized builds (Release/RelWithDebInfo) on non-MSVC toolchains.
option(SOUNDTABLE_TRACKER_ENABLE_AVX "Enable AVX/AVX2/FMA optimizations for soundtable-tracker (may reduce CPU compatibility)" OFF)

if (SOUNDTABLE_TRACKER_ENABLE_AVX AND NOT MSVC)
    include(CheckCXXCompilerFlag)

    set(SOUNDTABLE_TRACKER_AVX_FLAGS "")

    check_cxx_compiler_flag("-mavx" SOUNDTABLE_TRACKER_HAS_MAVX)
    if (SOUNDTABLE_TRACKER_HAS_MAVX)
        list(APPEND SOUNDTABLE_TRACKER_AVX_FLAGS "-mavx")
    endif()

    check_cxx_compiler_flag("-mavx2" SOUNDTABLE_TRACKER_HAS_MAVX2)
    if (SOUNDTABLE_TRACKER_HAS_MAVX2)
        list(APPEND SOUNDTABLE_TRACKER_AVX_FLAGS "-mavx2")
    endif()

    check_cxx_compiler_flag("-mfma" SOUNDTABLE_TRACKER_HAS_MFMA)
    if (SOUNDTABLE_TRACKER_HAS_MFMA)
        list(APPEND SOUNDTABLE_TRACKER_AVX_FLAGS "-mfma")
    endif()

    foreach(flag IN LISTS SOUNDTABLE_TRACKER_AVX_FLAGS)
        target_compile_options(soundtable-tracker PRIVATE
            "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${flag}>"
        )
    endforeach()
endif()

target_include_directories(soundtable-tracker PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/external/argparse/include
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack
)

target_link_libraries(soundtable-tracker PRIVATE ${OpenCV_LIBS})
