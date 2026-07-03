cmake_minimum_required(VERSION 3.8)
project(oni_nav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(oni_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(ACADOS_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../deps/acados_install" CACHE PATH "Acados install directory")
set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_DIR}/lib")

if(EXISTS "${ACADOS_LIB_DIR}/libacados.so")
  set(ONI_NAV_WITH_ACADOS ON)
  message(STATUS "Building with Acados NMPC from ${ACADOS_INSTALL_DIR}")
  add_subdirectory(c_generated_code)
else()
  set(ONI_NAV_WITH_ACADOS OFF)
  message(WARNING "Acados not found at ${ACADOS_LIB_DIR}; NMPC disabled")
endif()

set(ONI_NAV_CORE_SOURCES
  src/state_estimator.cpp
  src/phase_fsm.cpp
  src/failsafe.cpp
  src/target_localizer.cpp
  src/obstacle_model.cpp
  src/motion_controller.cpp
)

if(ONI_NAV_WITH_ACADOS)
  list(APPEND ONI_NAV_CORE_SOURCES src/nmpc_solver.cpp)
endif()

add_library(oni_nav_core ${ONI_NAV_CORE_SOURCES})

target_include_directories(oni_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(oni_nav_core PUBLIC Eigen3::Eigen)

if(ONI_NAV_WITH_ACADOS)
  target_include_directories(oni_nav_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
  )
  target_link_libraries(oni_nav_core PUBLIC oni_nmpc_acados)
  target_compile_definitions(oni_nav_core PUBLIC ONI_NAV_WITH_ACADOS=1)
endif()

add_executable(nav_controller_node src/nav_controller_node.cpp)

target_include_directories(nav_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  oni_msgs
)

target_link_libraries(nav_controller_node oni_nav_core)

set_target_properties(nav_controller_node PROPERTIES
  BUILD_RPATH "${ACADOS_LIB_DIR}"
  INSTALL_RPATH "${ACADOS_LIB_DIR}"
)

install(TARGETS oni_nav_core nav_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/mock_nav_inputs.py
  scripts/generate_nmpc_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_state_estimator test/test_state_estimator.cpp)
  target_link_libraries(test_state_estimator oni_nav_core)

  ament_add_gtest(test_phase_fsm test/test_phase_fsm.cpp)
  target_link_libraries(test_phase_fsm oni_nav_core)

  ament_add_gtest(test_target_localizer test/test_target_localizer.cpp)
  target_link_libraries(test_target_localizer oni_nav_core)

  ament_add_gtest(test_obstacle_model test/test_obstacle_model.cpp)
  target_link_libraries(test_obstacle_model oni_nav_core)

  ament_add_gtest(test_motion_controller test/test_motion_controller.cpp)
  target_link_libraries(test_motion_controller oni_nav_core)

  if(ONI_NAV_WITH_ACADOS)
    ament_add_gtest(test_nmpc_solver test/test_nmpc_solver.cpp)
    target_link_libraries(test_nmpc_solver oni_nav_core)
    set_target_properties(test_nmpc_solver PROPERTIES
      BUILD_RPATH "${ACADOS_LIB_DIR}"
    )
  endif()
endif()

ament_package()
