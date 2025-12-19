cmake_minimum_required(VERSION 3.20)

# Core JUCE application

if(NOT EXISTS "${JUCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "JUCE no encontrado en ${JUCE_DIR}. Consulta research/tech_stack.md para instrucciones.")
endif()

add_subdirectory(${JUCE_DIR} JUCE-build)

# System dependencies (Linux-specific)
if(UNIX AND NOT APPLE)
    # JUCE module dependencies as documented in docs/Linux Dependencies.md
    # juce_audio_devices, juce_audio_processors, juce_core, juce_graphics,
    # juce_gui_basics, juce_gui_extra

    # libcurl4-openssl-dev (juce_core / JUCE_USE_CURL)
    find_package(CURL REQUIRED)

    # Remaining libraries via pkg-config
    find_package(PkgConfig REQUIRED)

    set(RECTAI_JUCE_PKG_DEPS
        alsa            # libasound2-dev
        jack            # libjack-jackd2-dev
        freetype2       # libfreetype-dev / libfreetype6-dev
        fontconfig      # libfontconfig1-dev
        x11             # libx11-dev
        xcomposite      # libxcomposite-dev
        xcursor         # libxcursor-dev
        xext            # libxext-dev
        xinerama        # libxinerama-dev
        xrandr          # libxrandr-dev
        xrender         # libxrender-dev
    )

    foreach(dep IN LISTS RECTAI_JUCE_PKG_DEPS)
        pkg_check_modules(${dep}_PKG REQUIRED ${dep})
    endforeach()

    # WebKit2GTK for juce_gui_extra (web browser component)
    pkg_check_modules(WEBKIT2GTK_41 QUIET webkit2gtk-4.1)
    if(NOT WEBKIT2GTK_41_FOUND)
        # Fallback for older distros as per JUCE docs
        pkg_check_modules(WEBKIT2GTK_40 REQUIRED webkit2gtk-4.0)
    endif()

    # FluidSynth for SoundFont2 support in Sampleplay modules.
    pkg_check_modules(FLUIDSYNTH REQUIRED fluidsynth)

    # libzip for RTZ (ZIP archive) support in ReactableRtzLoader.
    pkg_check_modules(LIBZIP REQUIRED libzip)
endif()

# Librería de dominio (modelo de escena, módulos, etc.)

file(GLOB_RECURSE RECTAI_CORE_LIB_SOURCES CONFIGURE_DEPENDS
    src/core/*.cpp
    src/core/*.h
)

add_library(rectai-core-lib STATIC
    ${RECTAI_CORE_LIB_SOURCES}
)

target_include_directories(rectai-core-lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    $<$<AND:$<BOOL:${FLUIDSYNTH_FOUND}>,$<PLATFORM_ID:Linux>>:${FLUIDSYNTH_INCLUDE_DIRS}>
    $<$<BOOL:${LIBZIP_FOUND}>:${LIBZIP_INCLUDE_DIRS}>
)

if (MSVC)
    target_compile_options(rectai-core-lib PRIVATE /W4 /permissive-)
else()
    target_compile_options(rectai-core-lib PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(rectai-core-lib
        PUBLIC
            ${FLUIDSYNTH_LIBRARIES}
            $<$<BOOL:${LIBZIP_FOUND}>:${LIBZIP_LIBRARIES}>
    )
endif()

target_compile_definitions(rectai-core-lib
    PUBLIC
        $<$<BOOL:${LIBZIP_FOUND}>:RECTAI_HAVE_LIBZIP=1>
)


# Aplicación JUCE principal

set(PROJECT_NAME rectai-core)

juce_add_gui_app(${PROJECT_NAME}
    COMPANY_NAME "rectai"
    PRODUCT_NAME "RectaiTable"
    NEEDS_WEB_BROWSER TRUE
)

juce_generate_juce_header(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
        src/Main.cpp
        src/MainComponent.cpp
        src/MainComponent.h
        src/MainComponent_Paint.cpp
        src/MainComponent_Input.cpp
        src/MainComponent_Audio.cpp
        src/MainComponent_Atlas.cpp
        src/MainComponentHelpers.cpp
        src/MainComponentHelpers.h
        src/RectaiLookAndFeel.cpp
        src/AudioEngine.cpp
        src/AudioEngine.h
        src/TrackingOscReceiver.cpp
        src/TrackingOscReceiver.h
)

if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
    # Para la app JUCE principal evitamos -Wconversion, ya que provoca
    # muchos avisos ruidosos en el código de terceros (JUCE/Harfbuzz).
    # Seguimos usando un nivel alto de warnings en la librería de dominio
    # rectai-core-lib.
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        rectai-core-lib
        juce::juce_gui_extra
        juce::juce_audio_utils
        juce::juce_audio_formats
        juce::juce_dsp
        juce::juce_osc
    $<$<AND:$<BOOL:${CURL_FOUND}>,$<PLATFORM_ID:Linux>>:CURL::libcurl>
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)
