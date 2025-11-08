cmake_minimum_required(VERSION 3.10)

project(shiyan3)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cc")

set(SOURCES main.cc ${SRC_FILES})

add_executable(shiyan3 ${SOURCES})

target_include_directories(shiyan3 PRIVATE ${PROJECT_SOURCE_DIR}/include)

file(GLOB LIB_DIRS RELATIVE "${CMAKE_SOURCE_DIR}/library" "${CMAKE_SOURCE_DIR}/library/*")

foreach(lib ${LIB_DIRS})
  if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/library/${lib}/include")
    message(STATUS "添加头文件路径: ${CMAKE_SOURCE_DIR}/library/${lib}/include")
    target_include_directories(shiyan3 PRIVATE "${CMAKE_SOURCE_DIR}/library/${lib}/include")
  endif()
endforeach()
