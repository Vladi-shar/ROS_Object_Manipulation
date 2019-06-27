

cmake_minimum_required(VERSION 2.8.3)
project(mov_box)
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin and any catkin packages


 find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller_msgs
  manipulator_h_base_module_msgs
  qt_build
  cmake_modules
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robotis_controller_msgs manipulator_h_base_module_msgs qt_build cmake_modules eigen_conversions
  DEPENDS EIGEN3
)



include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(

  SYSTEM ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
)




catkin_package()
 

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

add_executable(mov_box_node ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(mov_box_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mov_box_node ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Eigen3_LIBRARIES})

