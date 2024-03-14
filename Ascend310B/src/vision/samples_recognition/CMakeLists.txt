cmake_minimum_required(VERSION 3.8)
project(samples_recognition)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

include_directories(
   ${OpenCV_INCLUDE_DIRS}
   ${AVCODEC_INCLUDE_DIR}
   ${INC_PATH}/runtime/include/
   include
)

link_directories(
    ${OpenCV_LIB_DIRS}
    ${AVCODEC_LIBRARY}
    ${LIB_PATH}
)

add_executable(recognition src/recognition.cpp)
target_include_directories(recognition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(recognition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  recognition
  "rclcpp"
  "cv_bridge"
  "image_transport"
  "OpenCV"
)

target_link_libraries(recognition ascendcl acl_dvpp stdc++ dl rt pthread acllite_dvpp_lite acllite_media acllite_om_execute acllite_common ${AVCODEC_LIBRARY})

install(TARGETS recognition
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
