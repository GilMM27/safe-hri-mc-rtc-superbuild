"""Install the opt-in bridge into the superbuild's external mc_mujoco source."""
import argparse
from pathlib import Path
import shutil


def install(source):
    source=Path(source); here=Path(__file__).parent
    cpp=source/'src/mj_sim.cpp'; header=source/'src/mj_sim_impl.h'
    c=cpp.read_text(); h=header.read_text()
    shutil.copyfile(here/'SmplhDistanceSnapshot.h', source/'src/SmplhDistanceSnapshot.h')
    shutil.copyfile(here/'SmplhDistanceWorker.h', source/'src/SmplhDistanceWorker.h')
    if 'smplh_bridge_.update' in c:
        if 'SmplhBridge smplh_bridge_' not in h: raise RuntimeError('Incomplete bridge patch')
        shutil.copyfile(here/'NativeSmplhPlayback.h',source/'src/SmplhBridge.h')
        if 'smplh_bridge_.renderDistanceOverlay' not in c:
            c=c.replace('  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);',
                        '  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);\n'
                        '  smplh_bridge_.renderDistanceOverlay(&scene);')
        if 'SMPL-H distance sample:' not in c:
            c=c.replace('    ImGui::Text("Wallclock time: %.2fs", wallclock);',
                        '    ImGui::Text("Wallclock time: %.2fs", wallclock);\n'
                        '    const auto smplh_distance = smplh_bridge_.snapshot();\n'
                        '    if(smplh_distance.ready)\n'
                        '      ImGui::Text("SMPL-H sample %llu | age %.1f ms | query %.1f ms",\n'
                        '                   static_cast<unsigned long long>(smplh_distance.sequence),\n'
                        '                   1000.0 * (data->time - smplh_distance.sample_simulation_time),\n'
                        '                   1000.0 * smplh_distance.worker_duration_seconds);\n'
                        '    else ImGui::Text("SMPL-H distance sample: waiting");')
        if 'smplh_store.assign<mc_mujoco::SmplhDistanceSnapshot>' not in c:
            c=c.replace('    if(!controller->run())',
                        '    auto & smplh_store = controller->controller().datastore();\n'
                        '    if(!smplh_store.has("SMPLH::DistanceSnapshot")) smplh_store.make<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot");\n'
                        '    smplh_store.assign<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot", smplh_bridge_.snapshot());\n'
                        '    if(!controller->run())')
            c=c.replace('      controller->run();',
                        '      auto & smplh_paused_store = controller->controller().datastore();\n'
                        '      if(!smplh_paused_store.has("SMPLH::DistanceSnapshot")) smplh_paused_store.make<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot");\n'
                        '      smplh_paused_store.assign<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot", smplh_bridge_.snapshot());\n'
                        '      controller->run();')
            cpp.write_text(c)
        cpp.write_text(c)
        cmake=source/'src/CMakeLists.txt'; text=cmake.read_text()
        text=text.replace('install(FILES mj_sim.h mj_configuration.h DESTINATION include/mc_mujoco)',
                          'install(FILES mj_sim.h mj_configuration.h SmplhDistanceSnapshot.h DESTINATION include/mc_mujoco)')
        text=text.replace('    mj_sim_impl.h\n', '    mj_sim_impl.h\n    SmplhDistanceSnapshot.h\n    SmplhDistanceWorker.h\n')
        if 'target_link_libraries(mc_mujoco_lib PRIVATE fcl)' not in text:
            text=text.replace('if(uitools_is_platform_ui_adapter)',
                              'target_link_libraries(mc_mujoco_lib PRIVATE fcl)\nif(uitools_is_platform_ui_adapter)')
        cmake.write_text(text)
        rootCmake=source/'CMakeLists.txt'; rootText=rootCmake.read_text()
        if 'find_package(fcl REQUIRED)' not in rootText:
            rootText=rootText.replace('find_package(mc_rtc REQUIRED)', 'find_package(mc_rtc REQUIRED)\nfind_package(fcl REQUIRED)')
        rootCmake.write_text(rootText); return
    replacements=[
        ('  mj_forward(model, data);','  mj_forward(model, data);\n  smplh_bridge_.update(model, data);'),
        ('  mj_step(model, data);','  mj_step(model, data);\n  mj_forward(model, data);\n  smplh_bridge_.update(model, data);'),
        ('  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);',
         '''#ifdef USE_UI_ADAPTER
  smplh_bridge_.upload(model, &platform_ui_adapter->mjr_context());
#else
  smplh_bridge_.upload(model, &context);
#endif
  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);
  smplh_bridge_.renderDistanceOverlay(&scene);''')]
    for before,after in replacements:
        if c.count(before)!=1: raise RuntimeError(f'Unsupported mc_mujoco source: {before}')
        c=c.replace(before,after)
    c=c.replace('    ImGui::Text("Wallclock time: %.2fs", wallclock);',
                '    ImGui::Text("Wallclock time: %.2fs", wallclock);\n'
                '    const auto smplh_distance = smplh_bridge_.snapshot();\n'
                '    if(smplh_distance.ready)\n'
                '      ImGui::Text("SMPL-H sample %llu | age %.1f ms | query %.1f ms",\n'
                '                   static_cast<unsigned long long>(smplh_distance.sequence),\n'
                '                   1000.0 * (data->time - smplh_distance.sample_simulation_time),\n'
                '                   1000.0 * smplh_distance.worker_duration_seconds);\n'
                '    else ImGui::Text("SMPL-H distance sample: waiting");')
    c=c.replace('    if(!controller->run())',
                '    auto & smplh_store = controller->controller().datastore();\n'
                '    if(!smplh_store.has("SMPLH::DistanceSnapshot")) smplh_store.make<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot");\n'
                '    smplh_store.assign<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot", smplh_bridge_.snapshot());\n'
                '    if(!controller->run())')
    c=c.replace('      controller->run();',
                '      auto & smplh_paused_store = controller->controller().datastore();\n'
                '      if(!smplh_paused_store.has("SMPLH::DistanceSnapshot")) smplh_paused_store.make<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot");\n'
                '      smplh_paused_store.assign<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot", smplh_bridge_.snapshot());\n'
                '      controller->run();')
    marker='struct MjSimImpl'
    # Insert instance member after the actual class opening, allowing inheritance.
    import re
    match=re.search(r'(?:class|struct) MjSimImpl\b[^;{]*\{',h)
    if not match: raise RuntimeError('Cannot locate MjSimImpl definition')
    h=h[:match.end()]+'\n  SmplhBridge smplh_bridge_;\n'+h[match.end():]
    h='#include "SmplhBridge.h"\n'+h
    cpp.write_text(c); header.write_text(h)
    shutil.copyfile(here/'NativeSmplhPlayback.h',source/'src/SmplhBridge.h')
    shutil.copyfile(here/'SmplhDistanceWorker.h',source/'src/SmplhDistanceWorker.h')
    cmake=source/'src/CMakeLists.txt'
    text=cmake.read_text()
    text=text.replace('install(FILES mj_sim.h mj_configuration.h DESTINATION include/mc_mujoco)',
                      'install(FILES mj_sim.h mj_configuration.h SmplhDistanceSnapshot.h DESTINATION include/mc_mujoco)')
    text=text.replace('    mj_sim_impl.h\n', '    mj_sim_impl.h\n    SmplhDistanceSnapshot.h\n    SmplhDistanceWorker.h\n')
    if 'target_link_libraries(mc_mujoco_lib PRIVATE fcl)' not in text:
        text=text.replace('if(uitools_is_platform_ui_adapter)',
                          'target_link_libraries(mc_mujoco_lib PRIVATE fcl)\nif(uitools_is_platform_ui_adapter)')
    cmake.write_text(text)
    rootCmake=source/'CMakeLists.txt'; rootText=rootCmake.read_text()
    if 'find_package(fcl REQUIRED)' not in rootText:
        rootText=rootText.replace('find_package(mc_rtc REQUIRED)', 'find_package(mc_rtc REQUIRED)\nfind_package(fcl REQUIRED)')
    rootCmake.write_text(rootText)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--source',required=True)
    install(p.parse_args().source)
