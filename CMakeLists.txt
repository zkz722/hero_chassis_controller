cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 代码规范检查
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    geometry_msgs
    nav_msgs
    tf
    dynamic_reconfigure
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# 生成动态参数代码
generate_dynamic_reconfigure_options(
    cfg/HeroChassisController.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs controller_interface hardware_interface pluginlib control_toolbox geometry_msgs nav_msgs tf dynamic_reconfigure
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

## 将控制器编译为动态库（插件）
add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
)

## 添加依赖
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}  # 确保你生成的头文件先被编译
    ${catkin_EXPORTED_TARGETS}
)

## 链接库
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############
# 安装动态库
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

# 安装 launch 和 config 文件夹 
install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)