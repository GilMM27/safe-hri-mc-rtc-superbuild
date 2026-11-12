file(GLOB extension_dirs CONFIGURE_DEPENDS "*")
foreach(extension_dir ${extension_dirs})
  if(EXISTS "${extension_dir}/CMakeLists.txt")
    addextension(${extension_dir})
  endif()
  if(extension_dir MATCHES ".cmake$" AND NOT extension_dir MATCHES "kinova_position_SSM_PFL_controller.cmake$")
    include(${extension_dir})
  endif()
endforeach()

# The controller's optional contact logging uses a header installed by mc_mujoco.
include(${CMAKE_CURRENT_SOURCE_DIR}/kinova_position_SSM_PFL_controller.cmake)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../kinova_controllers
  ${CMAKE_CURRENT_BINARY_DIR}/kinova_controllers
)

if(WITH_SMPLH_PLAYBACK)
  add_dependencies(KinovaHoldController mc_mujoco)
  add_dependencies(KinovaSweepController mc_mujoco)
endif()
