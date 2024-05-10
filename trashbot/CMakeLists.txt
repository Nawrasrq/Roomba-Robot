# Set the required cmake version and project name
cmake_minimum_required(VERSION 3.8)
project(trashbot)

# Check if c version is c99 and c++ is c++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add the following compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include dependencies specified in package.xml
find_package(ament_cmake REQUIRED)

# Includes linters and testing dependencies specified in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Remove below when a copyright and license are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # Remove below when a license and copyright are added in git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()