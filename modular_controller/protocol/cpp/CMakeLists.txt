cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(automobile VERSION 0.1.0)

# Include dir
include_directories(/usr/local/include)

# Src
AUX_SOURCE_DIRECTORY(src SRC_FILES)

# Headers
set(PROJECT_SOURCE_DIR ".")

# Source files
set(SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/protocol.hpp
    ${PROJECT_SOURCE_DIR}/protocol.cpp
)

# Set up such that XCode organizes the files correctly
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

# Add library
add_library(protocol SHARED ${SOURCE_FILES})

# Include directories
target_include_directories(protocol PRIVATE include/protocol)

# Install
install(TARGETS protocol DESTINATION lib)

# Install the headers
install(FILES include/protocol DESTINATION include)

# Create base directory
install(DIRECTORY include/protocol DESTINATION include)