cmake_minimum_required(VERSION 3.22)
project(BodycamMotion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 1. Fetch dependencies that preloader links as PRIVATE but exposes in its headers
FetchContent_Declare(
    boost_pfr
    GIT_REPOSITORY https://github.com/boostorg/pfr.git
    GIT_TAG boost-1.88.0
)
FetchContent_Declare(
    magic_enum
    GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
    GIT_TAG v0.9.7
)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(boost_pfr magic_enum nlohmann_json)

# 2. Fetch the preloader SDK
FetchContent_Declare(
    preloader_android
    GIT_REPOSITORY https://github.com/LiteLDev/preloader-android.git
    GIT_TAG 0.2.2)
FetchContent_MakeAvailable(preloader_android)

# Assuming you cloned the SDK into a folder named "sdk" or "LeviLance"
# Adjust the path to where you actually put the headers
include_directories(${CMAKE_SOURCE_DIR}/sdk/include) 

# OR if the headers are directly in a folder named "pl"
include_directories(${CMAKE_SOURCE_DIR}/sdk)

add_library(BodycamMotion SHARED 
src/BodycamMotion.cpp
src/Motion.hpp
src/Spring.hpp
)

# 3. Manually add the include directories for the PRIVATE dependencies
target_include_directories(BodycamMotion PRIVATE 
    ${boost_pfr_SOURCE_DIR}/include
    ${magic_enum_SOURCE_DIR}/include
    ${nlohmann_json_SOURCE_DIR}/include
)

target_link_libraries(BodycamMotion PRIVATE 
    preloader 
    android 
    log 
    EGL 
    GLESv3
)

target_compile_options(BodycamMotion PRIVATE -fvisibility=hidden -Wall -Wextra)
