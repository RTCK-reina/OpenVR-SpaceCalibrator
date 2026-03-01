# Platform abstraction layer: interfaces + POSIX/OpenVR concrete implementations
add_library(spacecal_platform
    src/console_logger.cpp
    src/chrono_clock.cpp
    src/file_config_store.cpp
    src/posix_ipc_transport.cpp
    src/posix_shared_memory.cpp
    src/openvr_runtime.cpp
)

target_include_directories(spacecal_platform
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lib          # picojson.h
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/openvr   # openvr.h
)

target_link_libraries(spacecal_platform
    PUBLIC
        spacecal_core
    PRIVATE
        rt    # shm_open / shm_unlink on Linux
)
