"""Summarize sampled SSM+PFL QP and MuJoCo residuals from an mc_rtc bin log."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--violation-tolerance', type=float, default=1e-3,
                        help='positive residual threshold in m/s')
    parser.add_argument('--acceleration-tolerance', type=float, default=1e-3,
                        help='positive temporal barrier QP residual threshold in m/s²')
    args = parser.parse_args()
    executable = shutil.which('mc_bin_utils')
    if not executable:
        parser.error('mc_bin_utils must be on PATH')
    listing = subprocess.check_output([executable, 'show', str(args.log)], text=True)
    available = set(re.findall(r'^- ([^ ]+) ', listing, re.MULTILINE))
    regions = sorted(key[len('SSMPFL_'):-len('_predicted_residual_mps')]
                     for key in available if key.startswith('SSMPFL_') and key.endswith('_predicted_residual_mps'))
    if not regions:
        parser.error('the log has no SSM+PFL regional diagnostics')
    pair_rows = sorted(key[:-len('_predicted_residual_mps')]
                       for key in available if key.startswith('SSMPFLPair_')
                       and key.endswith('_predicted_residual_mps'))
    row_keys = pair_rows or [f'SSMPFL_{region}' for region in regions]
    common = ['SSMPFL_active_pairs', 'SSMPFL_fallback', 'SSMPFL_solver_success',
              'SSMPFL_relaxed_pairs', 'SSMPFL_retreat_relaxed_pairs',
              'SSMPFL_full_qp_extra_slack', 'SSMPFL_full_qp_exceeded_legacy_cap',
              'SSMPFL_stage2_seconds',
              'SSMPFL_compute_seconds', 'SSMPFL_sample_age_seconds',
              'Kinova_sweep_joint_1_command', 'Kinova_sweep_joint_1_model',
              'Kinova_sweep_joint_1_measured', 'Kinova_sweep_joint_1_simulated',
              'Kinova_sweep_joint_1_simulated_velocity']
    suffixes = ['active', 'distance_m', 'closing_mps', 'combined_limit_mps',
                'predicted_residual_mps', 'actual_residual_mps']
    keys = [key for key in common if key in available]
    keys += [f'{row}_{suffix}' for row in row_keys for suffix in suffixes]
    speed_keys = [f'{row}_{suffix}' for row in row_keys
                  for suffix in ('robot_budget_mps', 'qp_next_robot_speed_mps')]
    if all(key in available for key in speed_keys):
        keys += speed_keys
    else:
        speed_keys = []
    relaxation_keys = [f'{row}_{suffix}' for row in row_keys
                       for suffix in ('relaxation_physical', 'retreat_relaxation_mps')]
    if all(key in available for key in relaxation_keys):
        keys += relaxation_keys
    else:
        relaxation_keys = []
    original_keys = [f'{row}_original_qp_residual' for row in row_keys]
    if all(key in available for key in original_keys):
        keys += original_keys
    else:
        original_keys = []
    acceleration_keys = [f'{row}_acceleration_residual_mps2' for row in row_keys]
    if all(key in available for key in acceleration_keys):
        keys += acceleration_keys
    else:
        acceleration_keys = []
    derivative_keys = [f'{row}_{suffix}' for row in row_keys
                       for suffix in ('derivative_valid', 'feature_switch')]
    if all(key in available for key in derivative_keys):
        keys += derivative_keys
    else:
        derivative_keys = []
    uncontrollable_keys = [f'{row}_uncontrollable' for row in row_keys]
    if all(key in available for key in uncontrollable_keys):
        keys += uncontrollable_keys
    else:
        uncontrollable_keys = []
    monitor_only_keys = [f'{row}_monitor_only' for row in row_keys]
    if all(key in available for key in monitor_only_keys):
        keys += monitor_only_keys
    else:
        monitor_only_keys = []
    with tempfile.TemporaryDirectory() as directory:
        exported = Path(directory) / 'ssm_pfl.csv'
        subprocess.run([executable, 'convert', '--in', str(args.log), '--out', str(exported),
                        '--entries', *keys], check=True)
        with exported.open(newline='') as stream:
            rows = list(csv.DictReader(stream, delimiter=';'))
    if not rows:
        parser.error('the log has no samples')
    def measured_pairs(row):
        return (int(row['SSMPFL_active_pairs'])
                + sum(int(row[key]) for key in uncontrollable_keys)
                + sum(int(row[key]) for key in monitor_only_keys))

    valid = [row for row in rows if measured_pairs(row) == len(row_keys)]
    result = {
        'samples': len(rows), 'samples_with_all_pairs': len(valid),
        'fallback_samples': sum(int(row['SSMPFL_fallback']) for row in rows),
        'solver_failed_samples': sum(not int(row['SSMPFL_solver_success']) for row in rows),
        'regions': len(regions), 'monitored_pairs': len(row_keys),
        'command_joint1_range_rad': [min(float(row['Kinova_sweep_joint_1_command']) for row in rows),
                                     max(float(row['Kinova_sweep_joint_1_command']) for row in rows)],
    }
    if not pair_rows:
        result['samples_with_all_regions'] = len(valid)
    model_key = 'Kinova_sweep_joint_1_model' if 'Kinova_sweep_joint_1_model' in keys else 'Kinova_sweep_joint_1_measured'
    result['model_joint1_range_rad'] = [min(float(row[model_key]) for row in rows),
                                        max(float(row[model_key]) for row in rows)]
    if 'Kinova_sweep_joint_1_simulated' in keys:
        actual_joints = [float(row['Kinova_sweep_joint_1_simulated']) for row in rows
                         if math.isfinite(float(row['Kinova_sweep_joint_1_simulated']))]
        if actual_joints:
            result['sampled_mujoco_joint1_range_rad'] = [min(actual_joints), max(actual_joints)]
        actual_velocities = [float(row['Kinova_sweep_joint_1_simulated_velocity']) for row in rows
                             if math.isfinite(float(row['Kinova_sweep_joint_1_simulated_velocity']))]
        if actual_velocities:
            result['sampled_mujoco_joint1_velocity_absmax_rad_s'] = max(map(abs, actual_velocities))
    observed = [float(row[f'{name}_actual_residual_mps'])
                for row in rows for name in row_keys
                if int(row[f'{name}_active']) or
                (uncontrollable_keys and int(row[f'{name}_uncontrollable'])) or
                (monitor_only_keys and int(row[f'{name}_monitor_only']))]
    if uncontrollable_keys:
        result['uncontrollable_pair_rows'] = sum(int(row[key]) for row in rows for key in uncontrollable_keys)
    if monitor_only_keys:
        result['monitor_only_pair_rows'] = sum(int(row[key]) for row in rows for key in monitor_only_keys)
    if relaxation_keys:
        result['max_approach_slack_mps'] = max(float(row[f'{name}_relaxation_physical'])
                                                for row in rows for name in row_keys)
        if 'SSMPFL_full_qp_extra_slack' in keys:
            result['max_full_qp_extra_slack_physical'] = max(
                float(row['SSMPFL_full_qp_extra_slack']) for row in rows)
        if 'SSMPFL_full_qp_exceeded_legacy_cap' in keys:
            result['full_qp_legacy_cap_exceedance_samples'] = sum(
                int(row['SSMPFL_full_qp_exceeded_legacy_cap']) for row in rows)
        if 'SSMPFL_stage2_seconds' in keys:
            result['max_stage2_seconds'] = max(float(row['SSMPFL_stage2_seconds']) for row in rows)
        result['max_retreat_slack_mps'] = max(float(row[f'{name}_retreat_relaxation_mps'])
                                               for row in rows for name in row_keys)
        result['retreat_relaxed_samples'] = sum(int(row['SSMPFL_retreat_relaxed_pairs']) > 0
                                                 for row in rows)
    if observed:
        result.update(observed_pair_rows=len(observed),
                      max_actual_residual_mps=max(observed),
                      actual_violations=sum(value > args.violation_tolerance for value in observed))
    if valid:
        predicted = [float(row[f'{name}_predicted_residual_mps'])
                     for row in valid for name in row_keys if int(row[f'{name}_active'])
                     and math.isfinite(float(row[f'{name}_predicted_residual_mps']))]
        acceleration = [float(row[f'{name}_acceleration_residual_mps2'])
                        for row in valid for name in row_keys if int(row[f'{name}_active'])
                        and acceleration_keys
                        and math.isfinite(float(row[f'{name}_acceleration_residual_mps2']))]
        result.update(max_compute_seconds=max(float(row['SSMPFL_compute_seconds']) for row in valid),
                      max_sample_age_seconds=max(float(row['SSMPFL_sample_age_seconds']) for row in valid))
        if predicted:
            result.update(max_predicted_residual_mps=max(predicted),
                          predicted_violations=sum(value > args.violation_tolerance for value in predicted))
        if original_keys:
            original = [float(row[f'{name}_original_qp_residual'])
                        for row in valid for name in row_keys if int(row[f'{name}_active'])]
            if original:
                result['max_original_qp_residual_mps'] = max(original)
                result['original_qp_violations'] = sum(value > args.violation_tolerance
                                                       for value in original)
        if speed_keys:
            speeds = [(float(row[f'{name}_qp_next_robot_speed_mps']),
                       float(row[f'{name}_robot_budget_mps']))
                      for row in valid for name in row_keys if int(row[f'{name}_active'])]
            if speeds:
                result['max_predicted_robot_point_speed_mps'] = max(speed for speed, _ in speeds)
                result['min_predicted_robot_point_speed_mps'] = min(speed for speed, _ in speeds)
                result['min_robot_point_budget_mps'] = min(budget for _, budget in speeds)
                result['max_point_speed_bound_excess_mps'] = max(abs(speed) - budget
                                                                 for speed, budget in speeds)
                zero_budget_speeds = [speed for speed, budget in speeds if budget <= 1e-9]
                if zero_budget_speeds:
                    result['min_predicted_speed_at_zero_budget_mps'] = min(zero_budget_speeds)
                    result['max_predicted_speed_at_zero_budget_mps'] = max(zero_budget_speeds)
        if acceleration:
            result.update(max_acceleration_residual_mps2=max(acceleration),
                          acceleration_violations=sum(value > args.acceleration_tolerance for value in acceleration))
    if derivative_keys and acceleration_keys and any(
            math.isfinite(float(row[key])) for row in rows for key in acceleration_keys):
        result['invalid_derivative_pair_samples'] = sum(
            not int(row[f'{name}_derivative_valid']) for row in rows for name in row_keys
            if math.isfinite(float(row[f'{name}_distance_m'])))
        result['feature_switch_pair_samples'] = sum(
            int(row[f'{name}_feature_switch']) for row in rows for name in row_keys
            if math.isfinite(float(row[f'{name}_distance_m'])))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
