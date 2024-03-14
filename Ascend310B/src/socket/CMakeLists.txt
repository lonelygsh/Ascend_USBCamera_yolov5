cmake_minimum_required(VERSION 3.8)
project(socket)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DENABLE_DVPP_INTERFACE)
set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend/ascend-toolkit/latest")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else()
    message(STATUS "set INC_PATH: ${INC_PATH}")
endif ()

set(LIB_PATH $ENV{NPU_HOST_LIB})
if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/ascend-toolkit/latest/runtime/lib64/stub")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else()
    message(STATUS "set LIB_PATH: ${LIB_PATH}")
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(shm_msgs REQUIRED)
find_package(Boost REQUIRED)
# find_package(nadjieb_mjpeg_streamer REQUIRED)
find_package(my_interfaces REQUIRED)

include_directories(
  include
)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

include_directories(
   ${INC_PATH}/runtime/include/
   include
)

link_directories(
    ${LIB_PATH}
)

add_executable(socket2PC src/socket2PC.cpp src/FFmpegVideoDecoder.c)
target_include_directories(socket2PC PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(socket2PC PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  socket2PC
  "rclcpp"
  "OpenCV"
  "my_interfaces"
)

target_include_directories(socket2PC PRIVATE ${shm_msgs_INCLUDE_DIRS})
target_link_libraries(socket2PC 
    shm_msgs::shm_msgs_image Boost::headers 
    # nadjieb_mjpeg_streamer::nadjieb_mjpeg_streamer
    ascendcl acl_cblas acl_dvpp acllite_common acllite_dvpp_lite stdc++
    avcodec avformat avutil swscale)

install(TARGETS socket2PC
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
