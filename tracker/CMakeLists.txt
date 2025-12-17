cmake_minimum_required(VERSION 3.20)

project(rectai-tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED core imgproc videoio highgui)

add_executable(rectai-tracker
    src/main.cpp
    src/OscSender.cpp
    src/TrackerTypes.h
    src/TrackerState.cpp
    src/TrackerEngine.cpp
    ${REACTIVISION_SOURCES}
)

# Place the executable in the top-level build directory so that
# Earthfile can export it from build/rectai-tracker
set_target_properties(rectai-tracker PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "\$ORIGIN/lib"
)

if (MSVC)
    target_compile_options(rectai-tracker PRIVATE /W4 /permissive-)
else()
    target_compile_options(rectai-tracker PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

# Optional AVX/AVX2 optimizations specifically for rectai-tracker.
# These flags are disabled by default to avoid generating binaries that
# cannot run on older CPUs. When enabled, they are only applied to
# optimized builds (Release/RelWithDebInfo) on non-MSVC toolchains.
option(RECTAI_TRACKER_ENABLE_AVX "Enable AVX/AVX2/FMA optimizations for rectai-tracker (may reduce CPU compatibility)" OFF)

if (RECTAI_TRACKER_ENABLE_AVX AND NOT MSVC)
    include(CheckCXXCompilerFlag)

    set(RECTAI_TRACKER_AVX_FLAGS "")

    check_cxx_compiler_flag("-mavx" RECTAI_TRACKER_HAS_MAVX)
    if (RECTAI_TRACKER_HAS_MAVX)
        list(APPEND RECTAI_TRACKER_AVX_FLAGS "-mavx")
    endif()

    check_cxx_compiler_flag("-mavx2" RECTAI_TRACKER_HAS_MAVX2)
    if (RECTAI_TRACKER_HAS_MAVX2)
        list(APPEND RECTAI_TRACKER_AVX_FLAGS "-mavx2")
    endif()

    check_cxx_compiler_flag("-mfma" RECTAI_TRACKER_HAS_MFMA)
    if (RECTAI_TRACKER_HAS_MFMA)
        list(APPEND RECTAI_TRACKER_AVX_FLAGS "-mfma")
    endif()

    foreach(flag IN LISTS RECTAI_TRACKER_AVX_FLAGS)
        target_compile_options(rectai-tracker PRIVATE
            "$<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${flag}>"
        )
    endforeach()
endif()

target_include_directories(rectai-tracker PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack
)

target_link_libraries(rectai-tracker PRIVATE ${OpenCV_LIBS})
