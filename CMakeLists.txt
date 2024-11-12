cmake_minimum_required(VERSION 3.16)
project(HybridAstar LANGUAGES CXX)

# Spécifier le standard C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_OSX_ARCHITECTURES "arm64")

# Ensure compile commands are generated
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Option for shared libraries (if needed)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Default to Release if no build type is set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set common flags (applies to all build types)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# Debug flags: Include debugging symbols, profiling tools, and sanitizers for memory checks
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer -fprofile-arcs -ftest-coverage")

# Release flags: Optimize for speed, disable debugging symbols, and prioritize performance
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Trouver Boost avec les composants requis
find_package(Boost REQUIRED)

if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost not found!")
endif()

# Trouver OMPL
find_package(ompl REQUIRED)

if(OMPL_FOUND)
    message(STATUS "OMPL found: ${OMPL_INCLUDE_DIRS}")
    include_directories(${OMPL_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OMPL not found!")
endif()

# find_package(FLANN REQUIRED)
#
# if(FLANN_FOUND)
#     message(STATUS "FLANN found: ${FLANN_INCLUDE_DIRS}")
#     include_directories(${FLANN_INCLUDE_DIRS})
# else()
#     message(FATAL_ERROR "FLANN not found!")
# endif()

# Fetch external dependencies using FetchContent
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)


# Spécifier le répertoire des bibliothèques OMPL
link_directories(${OMPL_LIBRARY_DIRS})

# Créer l'exécutable
add_executable(${PROJECT_NAME}
src/main.cpp
src/HybridAStar.cpp
src/Map.cpp
src/ReedsShepp.cpp
src/Node.cpp
src/PrecomputedData.cpp
)

# Inclure les répertoires
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
)

# Lier les bibliothèques nécessaires
target_link_libraries(${PROJECT_NAME} PRIVATE ${OMPL_LIBRARIES} sfml-graphics)
