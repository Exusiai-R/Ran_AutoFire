cmake_minimum_required(VERSION 3.8)
project(hnurm_autoaim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hnurm_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)

add_definitions(-DCMAKE_DEF_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS})

file(GLOB_RECURSE LMTD_SOURCES "include/aimer/base/*.cpp" "include/aimer/auto_aim/predictor/*.cpp" "include/base/*.cpp")
list(FILTER LMTD_SOURCES EXCLUDE REGEX ".*entry\\.cpp$")

add_executable(autoaim_mock_node src/autoaim_node.cpp ${LMTD_SOURCES})
ament_target_dependencies(autoaim_mock_node rclcpp hnurm_interfaces)
target_link_libraries(autoaim_mock_node ${OpenCV_LIBRARIES} Python3::Python fmt::fmt Ceres::ceres glog::glog)

install(TARGETS autoaim_mock_node DESTINATION lib/${PROJECT_NAME})
ament_package()
