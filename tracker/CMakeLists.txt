cmake_minimum_required(VERSION 3.20)

project(rectai-tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED core imgproc videoio)

add_executable(rectai-tracker
    src/main.cpp
    src/OscSender.cpp
)

# Place the executable in the top-level build directory so that
# Earthfile can export it from build/rectai-tracker
set_target_properties(rectai-tracker PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

if (MSVC)
    target_compile_options(rectai-tracker PRIVATE /W4 /permissive-)
else()
    target_compile_options(rectai-tracker PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

target_include_directories(rectai-tracker PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(rectai-tracker PRIVATE ${OpenCV_LIBS})
