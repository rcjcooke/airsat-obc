cmake_minimum_required(VERSION 3.10)
project(OBC_Flight_Software VERSION 1.0.0 LANGUAGES CXX)

# Force modern C++ version compliance rules
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add optimization and strict validation rules during compilation
add_compile_options(-O2 -Wall -Wextra)

# Define the source file arrays
set(SOURCES
    src/main.cpp
    src/AOCS.cpp
    src/AOCSController.cpp
)

# Define the output executable binary target name
add_executable(OBC ${SOURCES})

# Include directory indexing visibility controls
target_include_directories(OBC PRIVATE src)

# Link standard system libraries (Required for thread sleep and timing structures)
find_package(Threads REQUIRED)
target_link_libraries(OBC PRIVATE Threads::Threads)
