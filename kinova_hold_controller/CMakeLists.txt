cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME KinovaHoldController)
set(PROJECT_DESCRIPTION "Minimal posture-holding controller for the Kinova Gen3")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(
  etc/KinovaHoldController.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaHoldController.yaml
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaHoldController.yaml
        DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc
)

configure_file(
  etc/KinovaSweepController.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaSweepController.yaml
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaSweepController.yaml
        DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc
)
