# Driver component
add_library(spacecal_driver
    src/transform_applicator.cpp
    src/stub_pose_interceptor.cpp
)

target_include_directories(spacecal_driver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(spacecal_driver
    PUBLIC
        spacecal_core
        spacecal_protocol
)
