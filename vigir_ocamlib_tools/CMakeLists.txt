cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocamlib_tools)

find_package(cmake_modules REQUIRED)

FIND_PACKAGE(Eigen REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
)


FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules cv_bridge)
LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp cv_bridge
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Declare a cpp library
add_library(${PROJECT_NAME}  src/ocamlib_camera_model.cpp src/ocamlib_camera_model_cv1.cpp src/omni_camera.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(vigir_wide_angle_tools_node src/test.cpp)
target_link_libraries(vigir_wide_angle_tools_node ${catkin_LIBRARIES} ${PROJECT_NAME} )


install(TARGETS ${PROJECT_NAME} vigir_wide_angle_tools_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}  
  PATTERN "*.h"
)
