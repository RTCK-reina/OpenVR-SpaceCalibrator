# Protocol library: header-only, shared between client and driver
add_library(spacecal_protocol INTERFACE)

target_include_directories(spacecal_protocol
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_protocol
    INTERFACE
        spacecal_platform
)
