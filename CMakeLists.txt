project(Winster)
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-std=c++0x -fopenmp -msse4.1 -mavx2")

set(OpenMVG_DIR "/home/doctorant/libs/openMVG/build/openMVG_install/share/openMVG/cmake")

find_package(OpenMVG REQUIRED)
include_directories(${OPENMVG_INCLUDE_DIRS})


find_package(OpenCV 3.4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB MY_SRC
    "*.h"
    "*.cpp"
)

add_executable(${PROJECT_NAME} ${MY_SRC})

target_link_libraries(Winster ${OPENMVG_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

install(TARGETS Winster DESTINATION bin)
