# Platform abstraction layer: interfaces only (header-only library)
# Concrete implementations are compiled per-platform
add_library(spacecal_platform INTERFACE)

target_include_directories(spacecal_platform
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_platform
    INTERFACE
        spacecal_core
)
