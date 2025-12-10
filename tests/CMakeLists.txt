# Tests simples para rectai-core-lib

add_executable(rectai-core-tests
    scene_tests.cpp
)

if (MSVC)
    target_compile_options(rectai-core-tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(rectai-core-tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

target_link_libraries(rectai-core-tests
    PRIVATE
        rectai-core-lib
)

add_test(NAME rectai-core-tests COMMAND rectai-core-tests)

# Tracker tests: validate that TrackerEngine can detect amoeba fiducials
# (amoeba markers) and assign correct IDs based on the marker graphics.

find_package(OpenCV REQUIRED core imgproc imgcodecs)

add_executable(rectai-tracker-tests
    tracker_tests.cpp
    ../tracker/src/TrackerEngine.cpp
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/segment.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/fidtrackX.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/topologysearch.c
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack/treeidmap.cpp
)

if (MSVC)
    target_compile_options(rectai-tracker-tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(rectai-tracker-tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

target_include_directories(rectai-tracker-tests
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tracker/src
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack
)

target_link_libraries(rectai-tracker-tests
    PRIVATE
        ${OpenCV_LIBS}
)

target_compile_definitions(rectai-tracker-tests
    PRIVATE
        RECTAI_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME rectai-tracker-tests COMMAND rectai-tracker-tests)
