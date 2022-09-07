cmake_minimum_required(VERSION 3.10)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

option(WITH_COMEDI "With Comedi DAQ" OFF)
option(WITH_FLYCAPTURE "With FlyCapture2 SDK" OFF)
option(WITH_PYLON "With Pylon SDK" ON)
if(WITH_COMEDI)
  add_definitions(-DUSE_COMEDI)
endif()
if(WITH_FLYCAPTURE)
  add_definitions(-DUSE_FLYCAPTURE)
endif()
if(WITHPYLON)
  add_definitions(-DUSE_PYLON)
endif()

project(oculomatic)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package (OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE src include)
target_link_libraries(${PROJECT_NAME} ${CONAN_LIBS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
