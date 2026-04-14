cmake_minimum_required(VERSION 3.22)
project(hnurm_camera)

set(CMAKE_CXX_STANDARD 17)

if (${CMAKE_SYSTEM} MATCHES "Linux")
    file(GLOB SDK_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/*.so)
    file(GLOB THIRDPARTY_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/ThirdParty/*.so)
    set(SDK_PATH ${SDK_LIBS} ${THIRDPARTY_LIBS})
elseif (${CMAKE_SYSTEM} MATCHES "Darwin")
    file(GLOB SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/darwin/*.dylib)
endif ()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(hnurm_camera SHARED src/Camera.cpp src/camera_node.cpp)
target_link_libraries(hnurm_camera ${SDK_PATH})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN hnurm::CameraNode
    EXECUTABLE hnurm_camera_node
)

ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR
        INSTALL_TO_SHARE
        launch
        params
)
