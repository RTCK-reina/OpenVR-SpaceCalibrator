# UI layer (headers only for now, implementations depend on ImGui/GLFW/OpenGL)
add_library(spacecal_ui INTERFACE)

target_include_directories(spacecal_ui
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_ui
    INTERFACE
        spacecal_app
        spacecal_core
)
