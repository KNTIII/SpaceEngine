cmake_minimum_required(VERSION 3.20)
project(space_physics_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(space_physics SHARED
    src/PhysicsEngine.cpp
    src/PhysicsEngineCAPI.cpp
)
target_include_directories(space_physics PUBLIC include)
target_compile_definitions(space_physics PRIVATE SPACE_PHYSICS_BUILD_SHARED)
set_target_properties(space_physics PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    target_compile_options(space_physics PRIVATE /W4 /permissive-)
else()
    target_compile_options(space_physics PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

add_executable(space_physics_demo src/main.cpp)
target_link_libraries(space_physics_demo PRIVATE space_physics)

add_executable(space_physics_tests tests/PhysicsEngineTests.cpp)
target_link_libraries(space_physics_tests PRIVATE space_physics)

add_executable(space_physics_capi_tests tests/CAPI.t.cpp)
target_link_libraries(space_physics_capi_tests PRIVATE space_physics)

add_executable(space_physics_benchmark tests/benchmark.cpp)
target_link_libraries(space_physics_benchmark PRIVATE space_physics)

enable_testing()
add_test(NAME space_physics_tests COMMAND space_physics_tests)
add_test(NAME space_physics_capi_tests COMMAND space_physics_capi_tests)