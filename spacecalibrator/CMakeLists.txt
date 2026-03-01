cmake_minimum_required(VERSION 3.16)
project(SpaceCalibrator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(SPACECAL_BUILD_TESTS "Build unit tests" ON)
option(SPACECAL_BUILD_DRIVER "Build SteamVR driver" ON)
option(SPACECAL_BUILD_UI "Build overlay application" ON)

# Platform detection
include(cmake/Platform.cmake)

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Core library (pure math, no platform dependencies)
add_subdirectory(core)

# Protocol library (header-only)
add_subdirectory(protocol)

# Platform abstraction layer
add_subdirectory(platform)

# Application layer
add_subdirectory(app)

# UI layer
if(SPACECAL_BUILD_UI)
    add_subdirectory(ui)
endif()

# Driver component
if(SPACECAL_BUILD_DRIVER)
    add_subdirectory(driver)
endif()

# Tests
if(SPACECAL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
