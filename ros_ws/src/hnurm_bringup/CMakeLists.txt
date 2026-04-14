cmake_minimum_required(VERSION 3.8)
project(hnurm_bringup)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(sentry_manage_node
        src/sentry_manage_node.cpp
)

ament_auto_add_executable(compose_node
        src/compose_node.cpp
)
ament_target_dependencies(compose_node
        armor_solver
        hnurm_uart
        armor_detector
        hnurm_camera
)

ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR
        INSTALL_TO_SHARE
        params
        config
        launch
)
