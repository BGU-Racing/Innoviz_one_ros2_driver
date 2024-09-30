cmake_minimum_required(VERSION 3.8)
project(innovizapi_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall)
endif()

#Setting CXX STANDARD to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT DEFINED INVZ_DRIVER_VER OR INVZ_DRIVER_VER STREQUAL "")
  message(STATUS "INVZ_DRIVER_VER not defined, using default value x.x.x")
  set(INVZ_DRIVER_VER x.x.x)
else()
  message(STATUS "INVZ_DRIVER_VER defines as ${INVZ_DRIVER_VER}")
endif()
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_SOURCE_DIR}/../install")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

##adding the include dir
ament_export_include_directories(include)


# Set sources
set(innovizApiPath ${PROJECT_SOURCE_DIR}/innoviz_api/include)
set(libinnovizApi_Path ${PROJECT_SOURCE_DIR}/innoviz_api/lib/linux_x86/libinnovizApi.so)

message(STATUS "lib_Path: ${libinnovizApi_Path}")
message(STATUS "innovizApiPath: ${innovizApiPath}")

# Add executables
add_executable(Innoviz_one_FileReader src/Innoviz_one_FileReader.cpp src/ros_utils.cpp src/invz_utils.cpp src/InvzRosIReader.cpp src/InvzRosFileReader.cpp)
ament_target_dependencies(Innoviz_one_FileReader "rclcpp" "std_msgs" "pcl_ros" "pcl_msgs" "pcl_conversions" "Boost")  # Add actual dependencies here
target_link_libraries(Innoviz_one_FileReader ${libinnovizApi_Path})
target_include_directories(Innoviz_one_FileReader PRIVATE ${innovizApiPath} )
target_compile_definitions(Innoviz_one_FileReader PRIVATE INVZ_DRIVER_VERSION="${INVZ_DRIVER_VER}")

# add_executable(Innoviz_one_Device src/Innoviz_one_Device.cpp src/ros_utils.cpp src/invz_utils.cpp src/InvzRosIReader.cpp src/InvzRosDeviceInterface.cpp)
# ament_target_dependencies(Innoviz_one_Device "rclcpp" "sensor_msgs")  # Add actual dependencies here
# target_link_libraries(Innoviz_one_Device ${libinnovizApi_Path})
# target_include_directories(Innoviz_one_Device PRIVATE ${innovizApiPath})
# target_compile_definitions(Innoviz_one_Device PRIVATE INVZ_DRIVER_VERSION="${INVZ_DRIVER_VER}")

# add_executable(Innoviz_one_FileReader_test src/Innoviz_one_FileReader_test.cpp src/ros_utils.cpp)
# ament_target_dependencies(Innoviz_one_FileReader_test "rclcpp" "sensor_msgs")  # Add actual dependencies here
# target_include_directories(Innoviz_one_FileReader_test PRIVATE ${innovizApiPath})
# target_compile_definitions(Innoviz_one_FileReader_test PRIVATE INVZ_DRIVER_VERSION="${INVZ_DRIVER_VER}")

# Install targets
install(TARGETS
  Innoviz_one_FileReader
  # Innoviz_one_Device
  # Innoviz_one_FileReader_test
  DESTINATION lib/${PROJECT_NAME}
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
