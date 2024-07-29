cmake_minimum_required(VERSION 3.8)
project(plantfarm_ui)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## find dependencies

# set ros2 depends
set(ROS_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  # moveit_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  dsr_msgs2
  #dsr_hardware2
  plantfarm_msgs
  rclcpp_components
  ament_index_cpp
  realsense2
  cv_bridge
  pcl_conversions
)


find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${ROS_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL 1.10 REQUIRED)

find_package(OpenCV 4.5.0 REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(rviz_common REQUIRED)

## include directories

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

## to build
# get file directories

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
  yaml-cpp
  )

# to use qt5 ui

set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
#set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB_RECURSE UI
    "include/plantfarm_ui/*.ui"
)

qt5_wrap_ui (UI_HDRS  ${UI})

# build
# image pipeline
add_executable(pf_image_pipeline src/pf_image_pipeline/main.cpp src/pf_image_pipeline/pf_image_pipeline.cpp)
ament_target_dependencies(pf_image_pipeline ${ROS_INCLUDE_DEPENDS})
target_link_libraries(pf_image_pipeline ${EXTERNAL_DEPENDS_LIB} ${PCL_LIBRARIES} ) #-lopencv_core -lopencv_imgproc )

# ui
#add_executable(plantfarm_ui src/plantfarm_ui/main.cpp include/plantfarm_ui/plantfarm_ui.hpp src/plantfarm_ui/plantfarm_ui.cpp ${UI})
#ament_target_dependencies(plantfarm_ui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ROS_INCLUDE_DEPENDS})
#target_link_libraries(plantfarm_ui ${EXTERNAL_DEPENDS_LIB})
#target_include_directories(plantfarm_ui PRIVATE "include/${PROJECT_NAME}/")

# ui test
#add_executable(ui_test src/ui_test/main.cpp src/ui_test/ui_test.cpp)
#ament_target_dependencies(ui_test "rviz_common")
#target_link_libraries(ui_test Qt5::Widgets)

## install 

install(TARGETS pf_image_pipeline
  DESTINATION lib/${PROJECT_NAME}
)

#install(TARGETS ui_test
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)


# Macro for ament package
ament_package()
