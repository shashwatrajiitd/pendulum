cmake_minimum_required(VERSION 3.20)
project(pendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fno-fast-math -ffp-contract=off)
endif()

include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.2
    GIT_SHALLOW TRUE
)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()

add_library(Eigen3_headers INTERFACE)
target_include_directories(Eigen3_headers INTERFACE ${eigen_SOURCE_DIR})
add_library(Eigen3::Eigen ALIAS Eigen3_headers)

FetchContent_MakeAvailable(Catch2)

add_library(pendulum_core INTERFACE)
target_include_directories(pendulum_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pendulum_core INTERFACE Eigen3_headers)

if(EMSCRIPTEN)
    add_executable(pendulum_wasm src/api/wasm.cpp)
    target_link_libraries(pendulum_wasm PRIVATE pendulum_core)
    target_link_options(pendulum_wasm PRIVATE
        -sEXPORTED_FUNCTIONS=['_engine_create','_engine_destroy','_engine_advance','_engine_time','_engine_energy','_engine_energy_drift','_engine_initial_energy','_engine_get_n','_engine_get_state','_engine_get_positions','_malloc','_free']
        -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue']
        -sALLOW_MEMORY_GROWTH=1
        -sMODULARIZE=1
        -sEXPORT_NAME='PendulumModule'
        -sENVIRONMENT='web'
        -O3
    )
    set_target_properties(pendulum_wasm PROPERTIES
        OUTPUT_NAME "pendulum"
        SUFFIX ".js"
    )
else()
    enable_testing()
    add_subdirectory(tests)
endif()
