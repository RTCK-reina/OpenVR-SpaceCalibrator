# Core library: pure math, depends only on Eigen
add_library(spacecal_core
    src/kabsch_solver.cpp
    src/pose_averager.cpp
    src/isometry.cpp
)

target_include_directories(spacecal_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(spacecal_core
    PUBLIC
        Eigen3::Eigen
)

# Ensure no platform headers leak in
target_compile_definitions(spacecal_core
    PRIVATE
        SPACECAL_CORE_BUILD
)
