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

include(CTest)
if(BUILD_TESTING)
  add_executable(ssm_pfl_tvm_test tests/ssm_pfl_tvm.cpp)
  target_link_libraries(ssm_pfl_tvm_test PRIVATE mc_rtc::mc_control)
  add_test(NAME ssm_pfl_tvm COMMAND ssm_pfl_tvm_test)
  add_executable(kinova_tvm_smoke_test tests/kinova_tvm_smoke.cpp)
  target_link_libraries(kinova_tvm_smoke_test PRIVATE KinovaSweepController mc_rtc::mc_control)
  add_test(NAME kinova_tvm_smoke COMMAND kinova_tvm_smoke_test
           ${CMAKE_CURRENT_SOURCE_DIR}/../smplh_playback/KinovaSweepController.ssm_pfl.example.yaml)
  add_executable(kinova_position_ssm_pfl_smoke_test tests/kinova_position_ssm_pfl_smoke.cpp)
  target_link_libraries(kinova_position_ssm_pfl_smoke_test PRIVATE KinovaPositionSSMPFLController mc_rtc::mc_control)
  add_test(NAME kinova_position_ssm_pfl_smoke COMMAND kinova_position_ssm_pfl_smoke_test
           ${CMAKE_CURRENT_SOURCE_DIR}/../smplh_playback/KinovaPositionSSMPFLController.ssm_pfl.example.yaml)
endif()

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

configure_file(
  etc/KinovaPositionSSMPFLController.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaPositionSSMPFLController.yaml
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaPositionSSMPFLController.yaml
        DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc
)
