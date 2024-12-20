cmake_minimum_required(VERSION 3.10)

project(apps)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/apps.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
	${OpenCV_LIBS}
)
