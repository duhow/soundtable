cmake_minimum_required(VERSION 3.20)

# Core JUCE application

if(NOT EXISTS "${JUCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "JUCE no encontrado en ${JUCE_DIR}. Consulta research/tech_stack.md para instrucciones.")
endif()

add_subdirectory(${JUCE_DIR} JUCE-build)

set(PROJECT_NAME rectai-core)

juce_add_gui_app(${PROJECT_NAME}
    COMPANY_NAME "rectai"
    PRODUCT_NAME "RectaiTable"
)

juce_generate_juce_header(${PROJECT_NAME})

file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    src/*.cpp
    src/*.h
)

# Buenas prácticas de compilación generales
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /permissive-)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

# Incluir fuentes
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CORE_SOURCES}
)

# Dependencias de JUCE

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        juce::juce_gui_extra
        juce::juce_audio_utils
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)
