# Application layer
add_library(spacecal_app
    src/calibration_orchestrator.cpp
    src/device_manager.cpp
)

target_include_directories(spacecal_app
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_app
    PUBLIC
        spacecal_core
        spacecal_platform
        spacecal_protocol
)
