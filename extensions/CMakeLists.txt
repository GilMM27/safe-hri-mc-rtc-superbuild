file(GLOB extension_dirs CONFIGURE_DEPENDS "*")
foreach(extension_dir ${extension_dirs})
  if(EXISTS "${extension_dir}/CMakeLists.txt")
    addextension(${extension_dir})
  endif()
  if(extension_dir MATCHES ".cmake$")
    include(${extension_dir})
  endif()
endforeach()

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../kinova_hold_controller
  ${CMAKE_CURRENT_BINARY_DIR}/kinova_hold_controller
)

if(WITH_SMPLH_PLAYBACK)
  add_dependencies(KinovaHoldController mc_mujoco)
  add_dependencies(KinovaSweepController mc_mujoco)
endif()
