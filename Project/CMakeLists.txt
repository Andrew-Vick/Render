cmake_minimum_required(VERSION 3.10)
project(COMS336 LANGUAGES CXX)

set ( CMAKE_CXX_STANDARD          11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )
set ( CMAKE_CXX_EXTENSIONS        ON )

set ( SOURCE_ONE_WEEKEND
  src/main.cc
  src/color.h
  src/vec3.h
  src/ray.h
  src/hittable.h
  src/sphere.h
  src/hittable_list.h
  src/camera.h
  src/material.h
  src/rtweekend.h
  src/interval.h
  src/aabb.h
  src/quad.h
  src/bvh.h
  src/perlin.h
  src/rtw_stb_image.h
  src/texture.h
)

include_directories(src)

# Set the build type to Release
set(CMAKE_BUILD_TYPE Release)

add_executable(inOneWeekend ${EXTERNAL} ${SOURCE_ONE_WEEKEND})