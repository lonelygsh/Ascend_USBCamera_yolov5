cmake_minimum_required(VERSION 3.8)
project(face_recognition)

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
find_package(shm_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(my_interfaces REQUIRED)

include_directories(
   ${INC_PATH}/runtime/include/
   include
)

link_directories(
    ${LIB_PATH}
)

add_executable(face_recognition src/face_recognition.cpp src/dvppProcess.cpp)
target_include_directories(face_recognition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(face_recognition PRIVATE ${shm_msgs_INCLUDE_DIRS})
target_compile_features(face_recognition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    face_recognition
  "rclcpp"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "my_interfaces"
)

# target_include_directories(face_recognition PRIVATE ${AVCODEC_INCLUDE_DIRS})

target_link_libraries(face_recognition 
    ascendcl acl_cblas acl_dvpp acllite_common acllite_dvpp_lite stdc++ acllite_om_execute 
    shm_msgs::shm_msgs_image Boost::headers
    )

install(TARGETS face_recognition
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
