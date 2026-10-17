cmake_minimum_required(VERSION 3.0.2)
project(hikmicro_ptz_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)
find_package(nlohmann_json 3 REQUIRED)

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
  INCLUDE_DIRS include
  LIBRARIES hikmicro_isapi hikmicro_rtsp
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs geometry_msgs actionlib actionlib_msgs
    message_runtime cv_bridge image_transport
  DEPENDS CURL
)

# ISAPI client: HTTP digest, XML/JSON messages, PTZ, presets, auxiliaries, thermometry. No ROS.
add_library(hikmicro_isapi
  src/http_client.cpp
  src/isapi_xml.cpp
  src/radiometric.cpp
  src/camera.cpp
)
target_include_directories(hikmicro_isapi PUBLIC include ${CURL_INCLUDE_DIRS})
target_include_directories(hikmicro_isapi SYSTEM PRIVATE ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(hikmicro_isapi
  ${CURL_LIBRARIES} ${TINYXML2_LIBRARIES} nlohmann_json::nlohmann_json Threads::Threads)

# The node: PTZ (actions, presets, cmd_vel), auxiliaries, day/night, snapshots, thermometry
add_executable(hikmicro_ptz_node
  src/hikmicro_node.cpp
  src/ptz_interface.cpp
  src/aux_interface.cpp
  src/thermal_interface.cpp
)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(hikmicro_ptz_node hikmicro_isapi ${catkin_LIBRARIES})
add_dependencies(hikmicro_ptz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# RTSP capture (OpenCV, no ROS) and the camera node: one per stream, visible / thermal
add_library(hikmicro_rtsp src/rtsp_capture.cpp)
target_include_directories(hikmicro_rtsp PUBLIC include)
target_include_directories(hikmicro_rtsp SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hikmicro_rtsp ${OpenCV_LIBS} Threads::Threads)

add_executable(hikmicro_camera_node src/camera_node.cpp)
target_link_libraries(hikmicro_camera_node hikmicro_rtsp hikmicro_isapi ${catkin_LIBRARIES})
add_dependencies(hikmicro_camera_node ${catkin_EXPORTED_TARGETS})

install(TARGETS hikmicro_ptz_node hikmicro_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS hikmicro_isapi hikmicro_rtsp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_isapi_xml test/test_isapi_xml.cpp test/main.cpp)
  target_link_libraries(test_isapi_xml hikmicro_isapi)
  catkin_add_gtest(test_radiometric test/test_radiometric.cpp test/main.cpp)
  target_link_libraries(test_radiometric hikmicro_isapi)
  catkin_add_gtest(test_motion test/test_motion.cpp test/main.cpp)
  target_link_libraries(test_motion hikmicro_isapi)
  # end to end against test/fake_camera.py (HTTP, digest, simulated PTZ)
  catkin_add_gtest(test_camera_fake test/test_camera_fake.cpp test/main.cpp TIMEOUT 120)
  target_link_libraries(test_camera_fake hikmicro_isapi)
  target_compile_definitions(test_camera_fake PRIVATE
    FAKE_CAMERA_PY="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_camera.py")
  catkin_add_gtest(test_rtsp test/test_rtsp.cpp test/main.cpp TIMEOUT 60)
  target_link_libraries(test_rtsp hikmicro_rtsp)
endif()
