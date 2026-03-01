# Minimal Eigen3Config.cmake for local Eigen headers
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(EIGEN3_FOUND TRUE)
set(PACKAGE_VERSION "3.4.0")

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
    )
endif()
