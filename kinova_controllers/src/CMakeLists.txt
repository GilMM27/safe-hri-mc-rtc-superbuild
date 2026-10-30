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
