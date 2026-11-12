include(${CMAKE_CURRENT_LIST_DIR}/human_robot_distance_api.cmake)

set(SMPLH_FORCE_LOG_ARGS)
if(WITH_SMPLH_PLAYBACK)
  set(SMPLH_FORCE_LOG_ARGS mc_mujoco)
endif()

AddProject(kinova_position_SSM_PFL_controller
  GITHUB GilMM27/kinova_position_SSM_PFL_controller
  GIT_TAG origin/main
  CMAKE_ARGS -DWITH_SMPLH_CONTACT_LOGGING=${WITH_SMPLH_PLAYBACK}
  DEPENDS mc_rtc human_robot_distance_api ${SMPLH_FORCE_LOG_ARGS}
)
