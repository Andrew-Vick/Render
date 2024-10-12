cmake_minimum_required(VERSION 3.10)
project(COMS336 LANGUAGES CXX)

set ( CMAKE_CXX_STANDARD          14 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )
set ( CMAKE_CXX_EXTENSIONS        ON )

set ( SOURCE_ONE_WEEKEND
  src/aabb.h
  src/bvh.h
  src/camera.h 
  src/color.h 
  src/constant_medium.h  
  src/hittable_list.h 
  src/hittable.h 
  src/interval.h 
  src/main.cc 
  src/material.h 
  src/onb.h 
  src/pdf.h 
  src/perlin.h 
  src/quad.h 
  src/ray.h 
  src/rtw_stb_image.h 
  src/rtweekend.h 
  src/sphere.h 
  src/texture.h 
  src/vec3.h 
)

include_directories(src)

# Set the build type to Release
set(CMAKE_BUILD_TYPE debug)

add_executable(inOneWeekend ${EXTERNAL} ${SOURCE_ONE_WEEKEND})