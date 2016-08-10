cmake_minimum_required(VERSION 2.8.3)
project(simple_gesture)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  StringArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs message_runtime
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)


catkin_install_python(PROGRAMS scripts/filter.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/extract_hand.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/audio_feedback.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
