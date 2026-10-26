if(NOT WITH_Kinova)
  return()
endif()

include(FetchContent)

set(KINOVA_MJ_DESCRIPTION_COMMIT c96a32d28fb5da84da38c1da4d749e7a13212855)
set(KINOVA_MJ_DESCRIPTION_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco/kinova_gen3")

FetchContent_Declare(
  mujoco_menagerie
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
  GIT_TAG "${KINOVA_MJ_DESCRIPTION_COMMIT}"
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(mujoco_menagerie)

# The Menagerie model leaves MuJoCo's 2 ms default timestep implicit. Keep the
# installed model's physics step in sync with the 1 ms mc_rtc configurations.
set(KINOVA_MODEL_SOURCE "${mujoco_menagerie_SOURCE_DIR}/kinova_gen3/gen3.xml")
file(READ "${KINOVA_MODEL_SOURCE}" KINOVA_MODEL_XML)
set(KINOVA_MODEL_OPTION "<option integrator=\"implicitfast\"/>")
if(NOT KINOVA_MODEL_XML MATCHES "${KINOVA_MODEL_OPTION}")
  message(FATAL_ERROR "Kinova Menagerie model option changed; update the timestep override")
endif()
string(REPLACE "${KINOVA_MODEL_OPTION}"
  "<option timestep=\"0.001\" integrator=\"implicitfast\"/>"
  KINOVA_MODEL_XML "${KINOVA_MODEL_XML}")
set(KINOVA_MODEL_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gen3.xml")
file(WRITE "${KINOVA_MODEL_OUTPUT}" "${KINOVA_MODEL_XML}")

# Select the table environment only when Kinova is the main robot.
configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/kinova_ground.in.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/ground.yaml"
  @ONLY
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ground.yaml"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco")
install(FILES "${CMAKE_CURRENT_LIST_DIR}/kinova_table.xml"
  DESTINATION "${KINOVA_MJ_DESCRIPTION_DESTINATION}")

configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/kinova_mj_description.in.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/kinova.yaml"
  @ONLY
)
configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/kinova_mj_description.in.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/Kinova.yaml"
  @ONLY
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/kinova.yaml"
    "${CMAKE_CURRENT_BINARY_DIR}/Kinova.yaml"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco"
)

install(
  DIRECTORY "${mujoco_menagerie_SOURCE_DIR}/kinova_gen3/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco/kinova_gen3"
  PATTERN "gen3.xml" EXCLUDE
)
install(FILES "${KINOVA_MODEL_OUTPUT}"
  DESTINATION "${KINOVA_MJ_DESCRIPTION_DESTINATION}")
