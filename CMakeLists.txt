cmake_minimum_required(VERSION 3.8)
project(rsn_smacc2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
## find_package(smacc2_client_library REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system)

add_executable(rsn_smacc2_node src/main.cpp)

target_include_directories(rsn_smacc2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rsn_smacc2_node
  rclcpp
  smacc2
  std_srvs
  std_msgs
  Boost
)

target_link_libraries(rsn_smacc2_node
  Boost::thread
  Boost::system
)

install(TARGETS rsn_smacc2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)
  
ament_package()

