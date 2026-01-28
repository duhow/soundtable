# Tests simples para soundtable-core-lib

add_executable(soundtable-core-tests
    scene_tests.cpp
)

if (MSVC)
    target_compile_options(soundtable-core-tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(soundtable-core-tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

target_link_libraries(soundtable-core-tests
    PRIVATE
        soundtable-core-lib
)

add_test(NAME soundtable-core-tests COMMAND soundtable-core-tests)

# Tracker tests: validate that TrackerEngine can detect amoeba fiducials
# (amoeba markers) and assign correct IDs based on the marker graphics.

find_package(OpenCV REQUIRED core imgproc imgcodecs)

add_executable(soundtable-tracker-tests
    tracker_tests.cpp
    ../tracker/src/TrackerEngine.cpp
    ${REACTIVISION_SOURCES}
)

if (MSVC)
    target_compile_options(soundtable-tracker-tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(soundtable-tracker-tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

target_include_directories(soundtable-tracker-tests
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/tracker/src
    ${CMAKE_SOURCE_DIR}/reacTIVision/ext/libfidtrack
)

target_link_libraries(soundtable-tracker-tests
    PRIVATE
        ${OpenCV_LIBS}
)

target_compile_definitions(soundtable-tracker-tests
    PRIVATE
        SOUNDTABLE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME soundtable-tracker-tests COMMAND soundtable-tracker-tests)
