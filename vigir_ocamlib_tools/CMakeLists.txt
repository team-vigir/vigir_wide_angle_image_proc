cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocamlib_tools)



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
add_library(${PROJECT_NAME}  src/ocamlib_camera_model.cpp src/omni_camera.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
# add_executable(vigir_ocamlib_tools_node src/vigir_ocamlib_tools_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vigir_ocamlib_tools_node vigir_ocamlib_tools_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(vigir_ocamlib_tools_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vigir_ocamlib_tools vigir_ocamlib_tools_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_ocamlib_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
