# Application layer
add_library(spacecal_app
    src/calibration_orchestrator.cpp
    src/device_manager.cpp
    src/driver_bridge.cpp
    src/pose_collector.cpp
    src/profile_manager.cpp
)

target_include_directories(spacecal_app
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lib   # picojson.h
)

target_link_libraries(spacecal_app
    PUBLIC
        spacecal_core
        spacecal_platform
        spacecal_protocol
)
