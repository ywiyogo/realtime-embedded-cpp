cmake_minimum_required(VERSION 3.20)

# ============================================================================
# PROJECT CONFIGURATION
# ============================================================================

project(AerospaceRealTimeSystem
    VERSION 1.0.0
    DESCRIPTION "Real-Time Aerospace Flight Control System - C++23 Implementation"
    LANGUAGES CXX
)

# ============================================================================
# C++23 STANDARD AND COMPILER REQUIREMENTS
# ============================================================================

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure we have a compiler that supports C++23
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "13.0")
        message(FATAL_ERROR "GCC version 13.0 or higher is required for C++23 support")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "16.0")
        message(FATAL_ERROR "Clang version 16.0 or higher is required for C++23 support")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.30")
        message(FATAL_ERROR "MSVC version 19.30 or higher is required for C++23 support")
    endif()
endif()

# ============================================================================
# BUILD CONFIGURATION
# ============================================================================

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Build options
option(ENABLE_DEBUG_LOGGING "Enable debug logging output" OFF)
option(ENABLE_STATIC_ANALYSIS "Enable static analysis warnings" ON)
option(ENABLE_SANITIZERS "Enable runtime sanitizers for debugging" OFF)

# ============================================================================
# COMPILER FLAGS AND WARNINGS
# ============================================================================

# Base compiler flags for all configurations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Real-time system specific flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion -Wsign-conversion")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wnon-virtual-dtor -Woverloaded-virtual")

    # Real-time system specific warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstack-usage=8192")  # Warn if stack usage exceeds 8KB

    # Enable additional static analysis warnings if requested
    if(ENABLE_STATIC_ANALYSIS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2 -Wformat-security")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcast-align -Wcast-qual")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow")
    endif()
endif()

# Configuration-specific flags
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")

# ============================================================================
# SANITIZERS (for development and testing)
# ============================================================================

if(ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(SANITIZER_FLAGS "-fsanitize=address,undefined -fno-omit-frame-pointer")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${SANITIZER_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${SANITIZER_FLAGS}")
        message(STATUS "Enabled sanitizers: AddressSanitizer, UBSan")
    endif()
endif()

# ============================================================================
# COMPILE DEFINITIONS
# ============================================================================

# Debug logging
if(ENABLE_DEBUG_LOGGING)
    add_compile_definitions(DEBUG_LOGGING=1)
endif()

# Real-time system constraints
add_compile_definitions(
    AEROSPACE_REALTIME_SYSTEM=1
    MAX_STACK_USAGE=8192
    DETERMINISTIC_EXECUTION=1
)

# ============================================================================
# SOURCE FILES
# ============================================================================

set(AEROSPACE_SOURCES
    lib.cpp
    main.cpp
)

set(AEROSPACE_HEADERS
    lib.hpp
)

# ============================================================================
# LIBRARY TARGET
# ============================================================================

# Create static library for aerospace components
add_library(aerospace_lib STATIC lib.cpp lib.hpp)

# Set library properties
set_target_properties(aerospace_lib PROPERTIES
    OUTPUT_NAME "aerospace"
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

# Include directories
target_include_directories(aerospace_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# ============================================================================
# MAIN EXECUTABLE
# ============================================================================

add_executable(aerospace_system main.cpp)

# Link with the aerospace library
target_link_libraries(aerospace_system PRIVATE aerospace_lib)

# Set executable properties
set_target_properties(aerospace_system PROPERTIES
    OUTPUT_NAME "realtime_aerospace"
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

# ============================================================================
# TESTING SUPPORT
# ============================================================================

enable_testing()

# Simple test that runs the main program
add_test(NAME SystemDemonstration COMMAND aerospace_system)

# Set test properties
set_tests_properties(SystemDemonstration PROPERTIES
    TIMEOUT 60
    PASS_REGULAR_EXPRESSION "Flight Control System Demonstration Complete"
)

# ============================================================================
# INSTALLATION
# ============================================================================

# Install executable
install(TARGETS aerospace_system
    RUNTIME DESTINATION bin
    COMPONENT Runtime
)

# Install library
install(TARGETS aerospace_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    COMPONENT Development
)

# Install headers
install(FILES lib.hpp
    DESTINATION include/aerospace
    COMPONENT Development
)

# Install documentation
install(FILES README.md
    DESTINATION share/doc/aerospace
    COMPONENT Documentation
)

# ============================================================================
# STATUS REPORTING
# ============================================================================

message(STATUS "")
message(STATUS "=== Aerospace Real-Time System Configuration ===")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Debug Logging: ${ENABLE_DEBUG_LOGGING}")
message(STATUS "Static Analysis: ${ENABLE_STATIC_ANALYSIS}")
message(STATUS "Sanitizers: ${ENABLE_SANITIZERS}")
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "===============================================")
message(STATUS "")

# ============================================================================
# CUSTOM TARGETS FOR DEVELOPMENT
# ============================================================================

# Custom target for code formatting (if clang-format is available)
find_program(CLANG_FORMAT clang-format)
if(CLANG_FORMAT)
    add_custom_target(format
        COMMAND ${CLANG_FORMAT} -i -style=file lib.cpp lib.hpp main.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Formatting source code with clang-format"
    )
endif()

# Custom target for static analysis (if clang-tidy is available)
find_program(CLANG_TIDY clang-tidy)
if(CLANG_TIDY AND ENABLE_STATIC_ANALYSIS)
    add_custom_target(analyze
        COMMAND ${CLANG_TIDY} lib.cpp main.cpp -- -std=c++23
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Running static analysis with clang-tidy"
    )
endif()

# Custom target for quick build and run
add_custom_target(demo
    COMMAND ${CMAKE_COMMAND} --build . --target aerospace_system
    COMMAND ./bin/realtime_aerospace
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Building and running aerospace system demonstration"
    DEPENDS aerospace_system
)

# Build summary
add_custom_target(info
    COMMAND echo ""
    COMMAND echo "🛩️  Aerospace Real-Time System Build Information"
    COMMAND echo "=================================================="
    COMMAND echo "Build system: CMake ${CMAKE_VERSION}"
    COMMAND echo "C++ Standard: C++${CMAKE_CXX_STANDARD}"
    COMMAND echo "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
    COMMAND echo "Build type: ${CMAKE_BUILD_TYPE}"
    COMMAND echo ""
    COMMAND echo "Available targets:"
    COMMAND echo "  aerospace_system - Main executable"
    COMMAND echo "  aerospace_lib    - Static library"
    COMMAND echo "  demo            - Build and run demonstration"
    COMMAND echo "  test            - Run system tests"
    COMMAND echo "  format          - Format source code (if clang-format available)"
    COMMAND echo "  analyze         - Run static analysis (if clang-tidy available)"
    COMMAND echo ""
    COMMENT "Displaying build information"
)
