cmake_minimum_required(VERSION 2.8.3)
project(d_star_lite)

# Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp
             geometry_msgs
             costmap_2d
             nav_core
             base_local_planner
             global_planner)

catkin_package(CATKIN_DEPENDS roscpp
               geometry_msgs
               costmap_2d
               nav_core
               base_local_planner
               global_planner
               LIBRARIES d_star_lite_planner)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(d_star_lite_planner src/d_star_lite.cpp)
target_link_libraries(d_star_lite_planner ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/d_star_lite_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node d_star_lite_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES dstar_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
