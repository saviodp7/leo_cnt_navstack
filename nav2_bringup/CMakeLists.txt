cmake_minimum_required(VERSION 3.5)
project(nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

nav2_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY graphs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
endif()

ament_package()
