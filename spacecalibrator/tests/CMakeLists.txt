# Tests
# Uses Catch2 or GoogleTest -- configurable

include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# Core tests
add_executable(spacecal_core_tests
    core/kabsch_solver_test.cpp
    core/isometry_test.cpp
    core/pose_averager_test.cpp
    core/time_series_test.cpp
)

target_include_directories(spacecal_core_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../platform/include
)

target_link_libraries(spacecal_core_tests
    PRIVATE
        spacecal_core
        Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(spacecal_core_tests)

# App tests
add_executable(spacecal_app_tests
    app/orchestrator_test.cpp
)

target_include_directories(spacecal_app_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)

target_link_libraries(spacecal_app_tests
    PRIVATE
        spacecal_app
        Catch2::Catch2WithMain
)

catch_discover_tests(spacecal_app_tests)
