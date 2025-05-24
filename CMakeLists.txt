cmake_minimum_required(VERSION 3.5)
project(jim_control)

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

set(ACADO_INCLUDE_DIRS /usr/local/include/acado)
set(ACADO_LIBRARY_DIRS /usr/local/lib)
set(ACADO_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_auxiliary_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_qpoases_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_solver.c
)


include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ament_INCLUDE_DIRS}
    ${ACADO_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_action_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
)

link_directories(
    ${ACADO_LIBRARY_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(angles REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(mpc_libs
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_qpoases_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_auxiliary_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/acado_solver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/Bounds.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/Constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/CyclingManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/Indexlist.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/MessageHandling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/QProblem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/QProblemB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/SubjectTo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

target_include_directories(mpc_libs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/acado_mpc_export/qpoases/SRC
    
)

add_library(util_libs
    "src/goal_checker.cpp"
    "src/path_handler.cpp"
)

# add_library(acado_file
#     "src/acado_mpc.cpp"
# )   

set(DEPENDENCIES
    "rclcpp"
    "rclcpp_action"
    "nav_msgs"
    "rcutils"
    "sensor_msgs"
    "geometry_msgs"
    "std_msgs"
    "tf2"
    "tf2_ros"
    "nav2_util"
    "angles"
    "PCL"
    "pcl_conversions"
)



target_link_libraries(util_libs)
ament_target_dependencies(util_libs ${DEPENDENCIES} ${ament_LIBRARIES})

# add_executable(mpc_node src/mpc_node.cpp)
# target_link_libraries(mpc_node 
#     ${ament_LIBRARIES}
#     mpc_libs
#     Eigen3::Eigen
# )


add_executable(controller_server src/controller_server.cpp)

target_link_libraries(controller_server
    ${ament_LIBRARIES}
    mpc_libs
    util_libs
    Eigen3::Eigen
)

ament_target_dependencies(controller_server 
    rclcpp
    std_msgs
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    nav2_util
    angles
)
# ament_target_dependencies(controller_server
#     rclcpp
#     Eigen3::Eigen
#     std_msgs
#     tf2_ros
#     tf2_eigen
#     tf2_geometry_msgs
#     geometry_msgs
#     nav_msgs
#     nav2_msgs
#     nav2_util
#     angles
# )
# ament_target_dependencies(mpc_node rclcpp nav_msgs geometry_msgs)


# target_include_directories(${PROJECT_NAME}_lib ${CMAKE_CURRENT_SOURCE_DIR}/src)


# target_link_libraries(${PROJECT_NAME}_lib)
# ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES} ${ament_LIBRARIES} mpc_libs)
# add_executable(controller_server src/controller_server.cpp)
# target_link_libraries(controller_server ${PROJECT_NAME}_lib, mpc_libs)


install(TARGETS
  controller_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# install(
#     TARGETS controller_server
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
#     INCLUDES DESTINATION include
# )


# install(
#     TARGETS mpc_node
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
#     INCLUDES DESTINATION include
# )



# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
