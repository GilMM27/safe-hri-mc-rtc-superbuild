"""Install the opt-in bridge into the superbuild's external mc_mujoco source."""
import argparse
from pathlib import Path
import shutil

BRIDGE_MARKER = "// Installed by the superbuild SMPL-H bridge\n"


def cache_model_override(text):
    """Make the SMPL-H scene model follow the selected playback cache."""
    marker = '    auto object_cfg_path = get_robot_cfg_path(module);'
    if 'SMPLH_CACHE model' in text:
        return text
    if text.count(marker) != 1:
        raise RuntimeError('Unsupported mc_mujoco object loader')
    text = text.replace('#include <algorithm>', '#include <algorithm>\n#include <cstdlib>', 1)
    replacement = '''    // SMPLH_CACHE model: keep scene geometry and playback from the same cache.
    if(module == "smplh")
    {
      if(const char * cache = std::getenv("SMPLH_CACHE"))
      {
        fs::path cachePath(cache);
        if(cachePath.filename() == "smplh_cache.bin") cachePath = cachePath.parent_path();
        const auto xmlFile = cachePath / "human.xml";
        if(!fs::exists(xmlFile))
          mc_rtc::log::error_and_throw<std::runtime_error>(
              "[mc_mujoco] SMPLH_CACHE model cannot be found at {}", xmlFile.string());
        mjObjects[object.name] = xmlFile.string();
        continue;
      }
    }
'''+marker
    return text.replace(marker, replacement, 1)


def timing_readout(text):
    text=text.replace('pairs %llu/%llu",',
                      'pairs %llu/%llu\\nprep %.1f ms | collide %.1f ms | distance %.1f ms",')
    text=text.replace('pairs %llu/%llu | prep %.1f ms', 'pairs %llu/%llu\\nprep %.1f ms')
    text=text.replace('distance %.1f ms",', 'distance %.1f ms | SCH %.1f ms (%llu pruned) | %s",')
    text=text.replace('SCH %.1f ms (%llu pruned)",', 'SCH %.1f ms (%llu pruned) | %s",')
    text=text.replace('static_cast<unsigned long long>(smplh_distance.total_pairs));',
                      'static_cast<unsigned long long>(smplh_distance.total_pairs),\n'
                      '                   1000.0 * smplh_distance.preparation_duration_seconds,\n'
                      '                   1000.0 * smplh_distance.collision_duration_seconds,\n'
                      '                   1000.0 * smplh_distance.distance_duration_seconds);')
    text=text.replace('1000.0 * smplh_distance.distance_duration_seconds);',
                      '1000.0 * smplh_distance.distance_duration_seconds,\n'
                      '                   1000.0 * smplh_distance.sch_duration_seconds,\n'
                      '                   static_cast<unsigned long long>(smplh_distance.sch_pruned_pairs));')
    return text.replace('static_cast<unsigned long long>(smplh_distance.sch_pruned_pairs));',
                        'static_cast<unsigned long long>(smplh_distance.sch_pruned_pairs),\n'
                        '                   smplh_distance.approximate ? "approximate" : "exact");')


def limit_closest_speed_plot(source):
    """Keep the closest-pair speed plot on a rolling 30-second time axis."""
    plot = Path(source) / 'ext/mc_rtc-imgui/Plot.cpp'
    if not plot.exists():
        return
    text = plot.read_text()
    marker = '  // Rolling closest-pair speed plot installed by the superbuild.'
    if marker in text:
        return
    include = '#include "Plot.h"\n'
    append = '  plot.points.push_back({x, y});\n'
    if text.count(include) != 1 or text.count(append) != 1:
        raise RuntimeError('Unsupported mc_mujoco plot renderer')
    text = text.replace(include, include + '#include <algorithm>\n', 1)
    text = text.replace(append, '''  // Rolling closest-pair speed plot installed by the superbuild.
  if(title_ == "SSM+PFL closest robot approach speed (m/s)")
  {
    if(!plot.points.empty() && x < plot.points.back().x)
      plot.points.clear(); // Simulation time restarted.
    const double start = std::max(0.0, x - 30.0);
    const auto first = std::lower_bound(plot.points.begin(), plot.points.end(), start,
                                        [](const Point & point, double time) { return point.x < time; });
    plot.points.erase(plot.points.begin(), first);
    x_limits_ = {{start, std::max(30.0, x)}};
  }
  plot.points.push_back({x, y});
''', 1)
    plot.write_text(text)


def install(source):
    source=Path(source); here=Path(__file__).parent
    limit_closest_speed_plot(source)
    cpp=source/'src/mj_sim.cpp'; header=source/'src/mj_sim_impl.h'
    c=cache_model_override(cpp.read_text()); h=header.read_text()
    if BRIDGE_MARKER in c:
        shutil.copyfile(here/'SmplhDistanceWorker.h', source/'src/SmplhDistanceWorker.h')
        shutil.copyfile(here/'NativeSmplhPlayback.h', source/'src/SmplhBridge.h')
        return
    if 'smplh_bridge_.update' in c:
        raise RuntimeError('Existing mc_mujoco bridge is from an older checkout; use a clean mc_mujoco clone')
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
                '      ImGui::Text("SMPL-H sample %llu | age %.1f ms | query %.1f ms | pairs %llu/%llu",\n'
                '                   static_cast<unsigned long long>(smplh_distance.sequence),\n'
                '                   1000.0 * (data->time - smplh_distance.sample_simulation_time),\n'
                '                   1000.0 * smplh_distance.worker_duration_seconds,\n'
                '                   static_cast<unsigned long long>(smplh_distance.queried_pairs),\n'
                '                   static_cast<unsigned long long>(smplh_distance.total_pairs));\n'
                '    else ImGui::Text("SMPL-H distance sample: waiting");')
    c=c.replace('    if(!controller->run())',
                '    auto & smplh_store = controller->controller().datastore();\n'
                '    if(!smplh_store.has(human_robot_distance::datastoreKey)) smplh_store.make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey);\n'
                '    smplh_store.assign<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, smplh_bridge_.snapshot());\n'
                '    if(!controller->run())')
    c=c.replace('      controller->run();',
                '      auto & smplh_paused_store = controller->controller().datastore();\n'
                '      if(!smplh_paused_store.has(human_robot_distance::datastoreKey)) smplh_paused_store.make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey);\n'
                '      smplh_paused_store.assign<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, smplh_bridge_.snapshot());\n'
                '      controller->run();')
    marker='struct MjSimImpl'
    # Insert instance member after the actual class opening, allowing inheritance.
    import re
    match=re.search(r'(?:class|struct) MjSimImpl\b[^;{]*\{',h)
    if not match: raise RuntimeError('Cannot locate MjSimImpl definition')
    h=h[:match.end()]+'\n  SmplhBridge smplh_bridge_;\n'+h[match.end():]
    h='#include "SmplhBridge.h"\n'+h
    cpp.write_text(BRIDGE_MARKER + timing_readout(c)); header.write_text(h)
    shutil.copyfile(here/'NativeSmplhPlayback.h',source/'src/SmplhBridge.h')
    shutil.copyfile(here/'SmplhDistanceWorker.h',source/'src/SmplhDistanceWorker.h')
    cmake=source/'src/CMakeLists.txt'
    text=cmake.read_text()
    if '    SmplhDistanceWorker.h\n' not in text:
        text=text.replace('    mj_sim_impl.h\n', '    mj_sim_impl.h\n    SmplhDistanceWorker.h\n')
    if 'target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES}' not in text:
        if 'target_link_libraries(mc_mujoco_lib PRIVATE fcl)' in text:
            text=text.replace('target_link_libraries(mc_mujoco_lib PRIVATE fcl)',
                              'target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES})')
        else:
            text=text.replace('if(uitools_is_platform_ui_adapter)',
                              'target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES})\nif(uitools_is_platform_ui_adapter)')
    if 'target_include_directories(mc_mujoco_lib PRIVATE ${SCH_CORE_INCLUDE_DIRS})' not in text:
        text=text.replace('target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES})',
                          'target_include_directories(mc_mujoco_lib PRIVATE ${SCH_CORE_INCLUDE_DIRS})\n'
                          'target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES})')
    text=text.replace('target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES})',
                      'target_link_libraries(mc_mujoco_lib PRIVATE fcl ${SCH_CORE_LIBRARIES} human_robot_distance_api::human_robot_distance_api)')
    cmake.write_text(text)
    rootCmake=source/'CMakeLists.txt'; rootText=rootCmake.read_text()
    if 'find_package(fcl REQUIRED)' not in rootText:
        rootText=rootText.replace('find_package(mc_rtc REQUIRED)', 'find_package(mc_rtc REQUIRED)\nfind_package(fcl REQUIRED)')
    if 'find_package(sch-core REQUIRED)' not in rootText:
        rootText=rootText.replace('find_package(fcl REQUIRED)', 'find_package(fcl REQUIRED)\nfind_package(sch-core REQUIRED)')
    if 'find_package(human_robot_distance_api 2 CONFIG REQUIRED)' not in rootText:
        rootText=rootText.replace('find_package(mc_rtc REQUIRED)',
                                  'find_package(mc_rtc REQUIRED)\nfind_package(human_robot_distance_api 2 CONFIG REQUIRED)')
    rootCmake.write_text(rootText)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--source',required=True)
    install(p.parse_args().source)
