cmake_minimum_required(VERSION 2.8.3)
project(mynteye_ros_wrapper)
add_definitions(-std=c++11)


IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)


if(DEFINED ENV{MYNTEYE_SDK_ROOT})
  set(MYNTEYE_SDK_DIR $ENV{MYNTEYE_SDK_ROOT})
endif()
set(MYNT_SDK_LIB_DIR ${MYNTEYE_SDK_DIR}/lib)


find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    rosconsole
    sensor_msgs
    dynamic_reconfigure
    tf2_ros
    pcl_conversions
    nodelet
)


find_package(OpenCV REQUIRED)

find_package(CUDA ${CUDA_VERSION} REQUIRED)


catkin_package()


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${MYNTEYE_SDK_DIR}/include
)

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${MYNT_SDK_LIB_DIR})



set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
  ${OpenCV_LIBS}

  )



add_library(MYNTWrapper src/mynteye_ros_wrapper_nodelet.cpp)
target_link_libraries(MYNTWrapper
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${MYNT_SDK_LIB_DIR}/libmynteye_core.so
    ${MYNT_SDK_LIB_DIR}/libmynteye_features.so
    ${MYNT_SDK_LIB_DIR}/libmynteye_cudafeatures.so
    ${LINK_LIBRARIES})
add_dependencies(MYNTWrapper ${PROJECT_NAME}_gencfg)



add_executable(mynteye_wrapper_node src/mynteye_ros_wrapper_node.cpp)
target_link_libraries(mynteye_wrapper_node MYNTWrapper ${LINK_LIBRARIES})
add_dependencies(mynteye_wrapper_node ${PROJECT_NAME}_gencfg)

FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

install(TARGETS
  MYNTWrapper
  mynteye_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
