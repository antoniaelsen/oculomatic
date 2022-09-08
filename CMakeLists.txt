cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(oculomatic)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

option(WITH_COMEDI "With Comedi DAQ" OFF)
option(WITH_FLYCAPTURE "With FlyCapture2 SDK" OFF)
option(WITH_PYLON "With Pylon SDK" ON)

if(WITH_COMEDI)
  message("Using Comedi")
  add_definitions(-DUSE_COMEDI)

  SET(COMEDI_INSTALL "/usr/local/")
  FIND_FILE(COMEDILIB_FOUND "comedilib.h" ${COMEDI_INSTALL}/include)
  SET(COMEDI_INCLUDE_DIRS "${COMEDI_INSTALL}/include")
  SET(COMEDI_LINK_DIRS "${COMEDI_INSTALL}/lib")
  SET(COMEDI_LIBS "comedi")
  FIND_LIBRARY(COMEDI_LIBRARY NAMES comedilib comedi HINTS "${COMEDI_LINK_DIRS}")
endif()

if(WITH_FLYCAPTURE)
  message("Using FlyCapture")
  add_definitions(-DUSE_FLYCAPTURE)
endif()

if(WITH_PYLON)
  message("Using Pylon ${WITH_PYLON}")
  add_definitions(-DUSE_PYLON)
  find_package(pylon 7.1.0 REQUIRED)
endif()


file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE src include)
target_link_libraries(${PROJECT_NAME} ${CONAN_LIBS})


if(WITH_COMEDI)
  target_link_libraries(${PROJECT_NAME} ${COMEDI_LIBRARY})
endif()

if(WITH_FLYCAPTURE)
  target_link_libraries(${PROJECT_NAME} ${FLYCAPTURE2})
endif()

if(WITH_PYLON)
  target_link_libraries(${PROJECT_NAME} pylon::pylon)
endif()
