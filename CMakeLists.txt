project(ipm_auto_calib)
cmake_minimum_required(VERSION 2.8)
#SET(CMAKE_BUILD_TYPE DEBUG)
SET(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set("OpenCV_DIR" "/usr/local/share/OpenCV")
#set("OpenCV_DIR" "/usr/local2/share/OpenCV")
find_package( OpenCV REQUIRED )

set(PROJECT_ROOT ".")
include_directories(${PROJECT_ROOT}/include)
set(MODULE_SRC_ROOT ${PROJECT_ROOT}/src/)

set(MODULE_SRC_ROOT ${PROJECT_ROOT}/src/)
file(GLOB src_files ${MODULE_SRC_ROOT}/*.cpp)

file(GLOB src_video_files ${PROJECT_ROOT}/ipm_auto_calib_video_used_parameter.cpp)
#MESSAGE(${src_video_files})
add_executable(ipm_calib_video ${src_video_files} ${src_files})
target_link_libraries(ipm_calib_video ${OpenCV_LIBS} )


file(GLOB src_video_un_files ${PROJECT_ROOT}/ipm_auto_calib_video_unused_parameter.cpp)
#MESSAGE(${src_video_files})
add_executable(ipm_calib_video_un ${src_video_un_files} ${src_files})
target_link_libraries(ipm_calib_video_un ${OpenCV_LIBS} )