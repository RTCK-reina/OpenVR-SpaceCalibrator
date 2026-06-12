# Tests use a small local harness to keep configure/build independent of
# network-fetched test frameworks.

# Core tests
add_executable(spacecal_core_tests
    test_main.cpp
    core/kabsch_solver_test.cpp
    core/isometry_test.cpp
    core/pose_averager_test.cpp
    core/time_series_test.cpp
)

target_include_directories(spacecal_core_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../platform/include
)

target_link_libraries(spacecal_core_tests
    PRIVATE
        spacecal_core
)

add_test(NAME spacecal_core_tests COMMAND spacecal_core_tests)

# App tests
add_executable(spacecal_app_tests
    test_main.cpp
    app/orchestrator_test.cpp
    app/driver_bridge_test.cpp
    app/pose_collector_test.cpp
    app/profile_manager_test.cpp
)

target_include_directories(spacecal_app_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)

target_link_libraries(spacecal_app_tests
    PRIVATE
        spacecal_app
)

add_test(NAME spacecal_app_tests COMMAND spacecal_app_tests)
