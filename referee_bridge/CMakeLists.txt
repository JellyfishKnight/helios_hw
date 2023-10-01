cmake_minimum_required(VERSION 3.22.0)
project(referee_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY hardware/src
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  hardware/include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface 
  referee_bridge.xml
  )

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  )