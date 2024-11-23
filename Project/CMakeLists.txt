cmake_minimum_required ( VERSION 3.1.0...3.27.0 )
project(COMS336 LANGUAGES CXX)

# Set to C++11
set ( CMAKE_CXX_STANDARD          11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )
set ( CMAKE_CXX_EXTENSIONS        OFF )

# Source

set ( EXTERNAL
  src/external/stb_image.h
)

set ( SOURCE_ONE_WEEKEND
  src/main.cc
  src/aabb.h
  src/camera.h
  src/color.h
  src/constant_medium.h
  src/hittable.h
  src/hittable_list.h
  src/interval.h
  src/material.h
  src/onb.h
  src/pdf.h
  src/perlin.h
  src/quad.h
  src/ray.h
  src/thread_pool.h
  src/rtw_stb_image.h
  src/rtweekend.h
  src/sphere.h
  src/texture.h
  src/vec3.h
)

include_directories(src)

# Assimp paths (update if your installation is in a different location)
set(ASSIMP_INCLUDE_DIRS "/usr/local/include")
set(ASSIMP_LIBRARIES "/usr/local/lib/libassimp.dylib")

include_directories(${ASSIMP_INCLUDE_DIRS})

# Set the build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Add debug symbols


# Specific compiler flags below. We're not going to add options for all possible compilers, but if
# you're new to CMake (like we are), the following may be a helpful example if you're using a
# different compiler or want to set different compiler options.

message (STATUS "Compiler ID: " ${CMAKE_CXX_COMPILER_ID})
message (STATUS "Release flags: " ${CMAKE_CXX_FLAGS_RELEASE})
message (STATUS "Debug flags: " ${CMAKE_CXX_FLAGS_DEBUG})

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # /wd #### - Disable warning
    # /we #### - treat warning as error
    add_compile_options("/W4")      # Enable level-4 warnings
    add_compile_options("/we 4265") # Class has virtual functions, but its non-trivial destructor is not virtual
    add_compile_options("/we 5204") # Class has virtual functions, but its trivial destructor is not virtual
    add_compile_options("/wd 4100") # unreferenced formal parameter
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wnon-virtual-dtor) # Class has virtual functions, but its destructor is not virtual
    add_compile_options(-Wreorder) # Data member will be initialized after [other] data member
    add_compile_options(-Wmaybe-uninitialized) # Variable improperly initialized
    add_compile_options(-Wunused-variable) # Variable is defined but unused
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wnon-virtual-dtor) # Class has virtual functions, but its destructor is not virtual
    add_compile_options(-Wreorder) # Data member will be initialized after [other] data member
    add_compile_options(-Wsometimes-uninitialized) # Variable improperly initialized
    add_compile_options(-Wunused-variable) # Variable is defined but unused
endif()

# Executables
add_executable(inOneWeekend ${EXTERNAL} ${SOURCE_ONE_WEEKEND})
target_link_libraries(inOneWeekend ${ASSIMP_LIBRARIES})