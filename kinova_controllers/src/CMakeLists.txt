set(controller_SRC KinovaHoldController.cpp)

set(controller_HDR KinovaHoldController.h)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(
  ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DKinovaHoldController_EXPORTS"
)

add_controller(KinovaSweepController KinovaSweepController.cpp KinovaSweepController.h)
target_link_libraries(KinovaSweepController PRIVATE mc_rtc::mc_control_fsm)

add_controller(KinovaPositionSSMPFLController KinovaPositionSSMPFLController.cpp KinovaPositionSSMPFLController.h)
target_link_libraries(KinovaPositionSSMPFLController PRIVATE mc_rtc::mc_control_fsm)

add_controller(KinovaPositionSSMPFLPositionTaskController KinovaPositionSSMPFLPositionTaskController.cpp KinovaPositionSSMPFLPositionTaskController.h)
target_link_libraries(KinovaPositionSSMPFLPositionTaskController PRIVATE mc_rtc::mc_control_fsm)

foreach(controller KinovaHoldController KinovaSweepController KinovaPositionSSMPFLController KinovaPositionSSMPFLPositionTaskController)
  if(TARGET human_robot_distance_api::human_robot_distance_api)
    target_link_libraries(${controller} PUBLIC human_robot_distance_api::human_robot_distance_api)
  else()
    # The superbuild installs this ExternalProject after its own configure step.
    target_include_directories(${controller} PUBLIC "${CMAKE_INSTALL_PREFIX}/include")
  endif()
  if(TARGET human_robot_distance_api)
    add_dependencies(${controller} human_robot_distance_api)
  endif()
endforeach()
