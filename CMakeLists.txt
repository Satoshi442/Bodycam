cmake_minimum_required(VERSION 3.22)
project(BodycamMotion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    preloader_android
    GIT_REPOSITORY https://github.com/LiteLDev/preloader-android.git
    GIT_TAG 0.2.2)
FetchContent_MakeAvailable(preloader_android)

add_library(BodycamMotion SHARED src/BodycamMotion.cpp)
target_link_libraries(BodycamMotion PRIVATE preloader android log EGL GLESv3)
target_compile_options(BodycamMotion PRIVATE -fvisibility=hidden -Wall -Wextra)
