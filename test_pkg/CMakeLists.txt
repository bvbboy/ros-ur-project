cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
# SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ros/kinetic)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    ur_modern_driver
    ur_msgs
    weitu
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
  features2d
  calib3d
  )

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
# find_package(OPENCV 3 REQUIRED)

# message(STATUS "${OpenCV_LIBS}???")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    weitu
  DEPENDS
    EIGEN3
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /opt/iCentral/iCentral/include
)

LINK_DIRECTORIES(${LINK_DIRECTORIES}
    /opt/iCentral/iCentral/lib)

set(WEITU_LIBS
    MVSDK
    ImageConvert)


add_executable(testtest
  src/test.cpp
)
add_dependencies(testtest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testtest
  ${catkin_LIBRARIES}
)

add_executable(target_pub
  src/target_pub.cpp
)
add_dependencies(target_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_pub
  ${catkin_LIBRARIES}
)

add_executable(getCurrent
  src/getCurrent.cpp
)
add_dependencies(getCurrent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(getCurrent
  ${catkin_LIBRARIES}
)

add_executable(addCollision
  src/addCollision.cpp
)
add_dependencies(addCollision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(addCollision
  ${catkin_LIBRARIES}
)

add_executable(getPose
  src/getPose.cpp
)
add_dependencies(getPose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(getPose
  ${catkin_LIBRARIES}
)

add_executable(pick_place1
  src/pick_place1.cpp
)
add_dependencies(pick_place1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place1
  ${catkin_LIBRARIES}
)

add_executable(sub_findobject2d
  src/sub_findobject2d.cpp
)
add_dependencies(sub_findobject2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_findobject2d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
)

add_executable(pointcloud
  src/pointcloud.cpp
)
add_dependencies(pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
)

add_executable(dual_getCurrent
  src/dual_getCurrent.cpp
)
add_dependencies(dual_getCurrent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_getCurrent
  ${catkin_LIBRARIES}
)

add_executable(io_state
  src/io_state.cpp
)
add_dependencies(io_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(io_state
  ${catkin_LIBRARIES}
)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR})

# add_executable(dual_arm
#   src/dual_arm.cpp
# )

# add_dependencies(dual_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dual_arm
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES} 
#   ${WEITU_LIBS}
# )

# add_executable(dual_arm_moveit
#   src/dual_arm_moveit.cpp
# )
# add_dependencies(dual_arm_moveit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dual_arm_moveit
#   ${catkin_LIBRARIES}
# )

add_executable(find_screwhole
  ${FINDER_SRC}
  ${SEG_SRC}
  src/find_screwhole.cpp
)
add_dependencies(find_screwhole ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_screwhole
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(pres1
  ${FINDER_SRC}
  ${SEG_SRC}
  src/pres1.cpp
)
add_dependencies(pres1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pres1
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(pres2
  ${FINDER_SRC}
  ${SEG_SRC}
  src/pres2.cpp
)
add_dependencies(pres2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pres2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(find_screwhole_continue
  ${FINDER_SRC}
  ${SEG_SRC}
  src/find_screwhole_continue.cpp
)
add_dependencies(find_screwhole_continue ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_screwhole_continue
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(test_83
  ${FINDER_SRC}
  ${SEG_SRC}
  src/test_83.cpp
)
add_dependencies(test_83 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_83
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(mir
  src/mir.cpp
)
add_dependencies(mir ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mir
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)

add_executable(test_929
  src/test_929.cpp
)
add_dependencies(test_929 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_929
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${WEITU_LIBS}
)
