cmake_minimum_required(VERSION 3.0.2)
project(hikmicro_ptz_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)
find_package(nlohmann_json 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hikmicro_isapi
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

install(TARGETS hikmicro_isapi
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
endif()
