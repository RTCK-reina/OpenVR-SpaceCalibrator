# Platform abstraction interfaces. App/core code should depend on this target,
# not on concrete OS/OpenVR implementations.
add_library(spacecal_platform INTERFACE)

target_include_directories(spacecal_platform
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_platform
    INTERFACE
        spacecal_core
)

# POSIX-backed concrete implementations.
add_library(spacecal_platform_posix
    src/console_logger.cpp
    src/chrono_clock.cpp
    src/file_config_store.cpp
    src/posix_ipc_transport.cpp
    src/posix_shared_memory.cpp
)

target_include_directories(spacecal_platform_posix
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(spacecal_platform_posix
    PUBLIC
        spacecal_platform
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(spacecal_platform_posix
        PRIVATE
            rt    # shm_open / shm_unlink on Linux
    )
endif()

# OpenVR-backed concrete runtime implementation.
add_library(spacecal_platform_openvr
    src/openvr_runtime.cpp
)

target_include_directories(spacecal_platform_openvr
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/openvr   # openvr.h
)

target_link_libraries(spacecal_platform_openvr
    PUBLIC
        spacecal_platform
)
