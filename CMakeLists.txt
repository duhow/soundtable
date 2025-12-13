cmake_minimum_required(VERSION 3.20)

project(rectai-table VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Opciones generales del proyecto
option(RECTAI_ENABLE_TRACKER "Build the tracker service" ON)
option(RECTAI_BUILD_TESTS "Build C++ tests" ON)

# Fuentes de la librería externa reacTIVision/libfidtrack que generan
# muchos warnings de conversion/sign-conversion/float-conversion.
# Los tratamos como código de terceros y desactivamos esos avisos solo
# para estos ficheros, manteniendo un nivel de warnings alto en el
# código propio.
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

include(CTest)

# Ruta para JUCE como submódulo (esperado en JUCE/)
set(JUCE_DIR "${CMAKE_SOURCE_DIR}/JUCE" CACHE PATH "Path to JUCE source directory")

if (NOT EXISTS "${JUCE_DIR}/CMakeLists.txt")
    message(WARNING "JUCE no encontrado en ${JUCE_DIR}. Añade JUCE como submódulo: git submodule add https://github.com/juce-framework/JUCE.git JUCE")
endif()

add_subdirectory(core)

if(RECTAI_ENABLE_TRACKER)
    add_subdirectory(tracker)
endif()

if(RECTAI_BUILD_TESTS)
    add_subdirectory(tests)
endif()