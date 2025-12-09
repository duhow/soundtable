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
