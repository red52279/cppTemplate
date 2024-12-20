cmake_minimum_required(VERSION 3.10)

project(main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_subdirectory(include/camera_driver)
add_subdirectory(include/vcom)
add_subdirectory(include/apps)

include_directories(
	${PROJECT_SOURCE_DIR}/include/vcom/include
	${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
	src/main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include/vcom/include
	${PROJECT_SOURCE_DIR}/include/camera_driver/include
	${PROJECT_SOURCE_DIR}/include/apps/include
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	camera_driver
	vcom
	apps
)


add_executable(cameraCalibrate
	src/cameraCalibrate.cpp
)

target_include_directories(cameraCalibrate PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include/camera_driver/include
)

target_link_libraries(cameraCalibrate
	${OpenCV_LIBS}
	camera_driver
)

