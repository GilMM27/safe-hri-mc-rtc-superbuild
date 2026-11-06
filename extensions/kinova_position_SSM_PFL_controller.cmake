include(${CMAKE_CURRENT_LIST_DIR}/human_robot_distance_api.cmake)

AddProject(kinova_position_SSM_PFL_controller
  GITHUB GilMM27/kinova_position_SSM_PFL_controller
  GIT_TAG origin/main
  DEPENDS mc_rtc human_robot_distance_api
)
