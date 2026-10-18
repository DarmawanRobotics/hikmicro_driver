cmake_minimum_required(VERSION 3.0.2)
project(hikmicro_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  actionlib_msgs
  message_generation
)

add_message_files(FILES
  PtzPreset.msg
  PtzState.msg
  ThermalStats.msg
)
add_service_files(FILES
  GetPresets.srv
  GetTemperature.srv
  RawRequest.srv
  SetAux.srv
  SetDayNight.srv
  SetPreset.srv
  Snapshot.srv
)
add_action_files(FILES
  GotoPreset.action
  MoveTo.action
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs actionlib_msgs message_runtime
)
