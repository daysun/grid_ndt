cmake_minimum_required(VERSION 2.8)
project(grid_ndt)

find_package(catkin REQUIRED COMPONENTS
 sensor_msgs tf
message_generation geometry_msgs std_msgs
cmake_modules
visualization_msgs
  roscpp
  sensor_msgs
  nodelet
  pcl_ros
)
find_package(octomap REQUIRED)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

FIND_PACKAGE(OpenGL)
FIND_PACKAGE(Qt4)
IF (OpenGL-NOTFOUND OR Qt4-NOTFOUND)
        MESSAGE ( "OpenGL and QT4 are required for octovis but could not be found.")
ELSE()
        FIND_PACKAGE(QGLViewer)
        IF(QGLViewer_FOUND)
          SET( BUILD_VIEWER 1)
        ELSE()
          MESSAGE ( "\n")
        MESSAGE ( "libQGLViewer could not be found or generated.")
        ENDIF()
ENDIF()

#openmp
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  DEPENDS sensor_msgs tf  message_runtime geometry_msgs std_msgs)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_libraries( ${catkin_LIBRARIES})

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

#publish the point
add_executable(publishPcd
 src/publisher.cpp)
target_link_libraries(publishPcd
${PCL_LIBRARIES} )

#publish change
add_executable(publishChange
 src/publishChange.cpp)
target_link_libraries(publishChange
${PCL_LIBRARIES} )

#publish del change
#add_executable(publishDel
# src/publishDel.cpp)
#target_link_libraries(publishDel
#${PCL_LIBRARIES} )

#receive the point and pocess the data
add_executable(receivePcd
     include/Stopwatch.h
     include/Vec3.h
     include/map2D.h
     include/robot.h
 src/receiver.cpp)
target_link_libraries(receivePcd
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )

#test
add_executable(txt2pcd_
 src/test/txt2pcd.cpp)
target_link_libraries(txt2pcd_
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )
add_executable(bag2pcd_
 src/test/bag2pcd.cpp)
target_link_libraries(bag2pcd_
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )
add_executable(bag2colorpcd_
 src/test/bag2colorpcd.cpp)
target_link_libraries(bag2colorpcd_
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )
add_executable(slopeTest
 src/test/explainSlope.cpp)
target_link_libraries(slopeTest
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )

add_executable(pose2pcd_
 src/test/pose2pcd.cpp)
target_link_libraries(pose2pcd_
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )
add_executable(unikob2pcd_
 src/test/unikob2pcd.cpp)
target_link_libraries(unikob2pcd_
${PCL_LIBRARIES}
 ${OpenCV_LIBS} )
add_executable(readfile_
 src/test/readfile.cpp)

#kitti to pcd
#ADD_EXECUTABLE(kitti2pcd src/test/kitti2pcd.cpp)
#TARGET_LINK_LIBRARIES(kitti2pcd ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#ADD_EXECUTABLE(kittitrackletextract src/test/tracklets.h src/test/kittitrackletextract.cpp )
#TARGET_LINK_LIBRARIES(kittitrackletextract ${Boost_LIBRARIES} ${PCL_LIBRARIES})

#install(TARGETS ${PROJECT_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/octomap_ros/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/octomap_ros/
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)



