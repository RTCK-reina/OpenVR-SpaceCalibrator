# Platform detection and configuration

if(WIN32)
    set(SPACECAL_PLATFORM "win32")
    add_compile_definitions(SPACECAL_PLATFORM_WIN32)
elseif(UNIX AND NOT APPLE)
    set(SPACECAL_PLATFORM "posix")
    add_compile_definitions(SPACECAL_PLATFORM_POSIX)
elseif(APPLE)
    set(SPACECAL_PLATFORM "posix")
    add_compile_definitions(SPACECAL_PLATFORM_POSIX)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

message(STATUS "SpaceCalibrator platform: ${SPACECAL_PLATFORM}")
