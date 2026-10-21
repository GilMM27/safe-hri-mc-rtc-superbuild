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
              'SSMPFL_compute_seconds', 'SSMPFL_sample_age_seconds',
              'Kinova_sweep_joint_1_command', 'Kinova_sweep_joint_1_model',
              'Kinova_sweep_joint_1_measured', 'Kinova_sweep_joint_1_simulated',
              'Kinova_sweep_joint_1_simulated_velocity']
    suffixes = ['active', 'closing_mps', 'combined_limit_mps',
                'predicted_residual_mps', 'actual_residual_mps']
    keys = [key for key in common if key in available]
    keys += [f'{row}_{suffix}' for row in row_keys for suffix in suffixes]
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
    if observed:
        result.update(observed_pair_rows=len(observed),
                      max_actual_residual_mps=max(observed),
                      actual_violations=sum(value > args.violation_tolerance for value in observed))
    if valid:
        predicted = [float(row[f'{name}_predicted_residual_mps'])
                     for row in valid for name in row_keys if int(row[f'{name}_active'])]
        result.update(max_compute_seconds=max(float(row['SSMPFL_compute_seconds']) for row in valid),
                      max_sample_age_seconds=max(float(row['SSMPFL_sample_age_seconds']) for row in valid))
        if predicted:
            result.update(max_predicted_residual_mps=max(predicted),
                          predicted_violations=sum(value > args.violation_tolerance for value in predicted))
    contact_fields = ('normal_force_n', 'sampled_relative_speed_mps', 'sampled_closing_mps')
    contact_keys = [f'SMPLH_contact_{region}_{field}'
                    for region in regions for field in contact_fields]
    if all(key in available for key in contact_keys):
        def finite_or_none(value):
            number = float(value)
            return number if math.isfinite(number) else None

        contact_keys += ['t', 'SSMPFL_sample_age_seconds', 'SSMPFL_solver_success',
                         'SSMPFL_at_speed_bound']
        contact_keys += [f'SSMPFL_{region}_{field}' for region in regions
                         for field in ('distance_m', 'robot_approach_mps',
                                       'human_approach_mps', 'active', 'binding')]
        with tempfile.TemporaryDirectory() as directory:
            exported = Path(directory) / 'contacts.csv'
            subprocess.run([executable, 'convert', '--in', str(args.log), '--out', str(exported),
                            '--entries', *contact_keys], check=True)
            contact_samples = 0
            first_contact = None
            with exported.open(newline='') as stream:
                for row in csv.DictReader(stream, delimiter=';'):
                    touching = [(region, float(row[f'SMPLH_contact_{region}_normal_force_n']))
                                for region in regions
                                if float(row[f'SMPLH_contact_{region}_normal_force_n']) > 0]
                    if not touching:
                        continue
                    contact_samples += 1
                    if first_contact is not None:
                        continue
                    region, force = max(touching, key=lambda item: item[1])
                    first_contact = {
                        'time_s': float(row['t']), 'region': region,
                        'normal_force_n': force,
                        'sampled_relative_speed_mps': finite_or_none(row[f'SMPLH_contact_{region}_sampled_relative_speed_mps']),
                        'sampled_closing_mps': finite_or_none(row[f'SMPLH_contact_{region}_sampled_closing_mps']),
                        'sampled_robot_approach_mps': finite_or_none(row[f'SSMPFL_{region}_robot_approach_mps']),
                        'sampled_human_approach_mps': finite_or_none(row[f'SSMPFL_{region}_human_approach_mps']),
                        'sampled_distance_m': finite_or_none(row[f'SSMPFL_{region}_distance_m']),
                        'distance_sample_age_s': finite_or_none(row['SSMPFL_sample_age_seconds']),
                        'qp_solver_success': bool(int(row['SSMPFL_solver_success'])),
                        'qp_row_active': bool(int(row[f'SSMPFL_{region}_active'])),
                        'qp_row_binding': bool(int(row[f'SSMPFL_{region}_binding'])),
                        'qp_any_row_binding': bool(int(row['SSMPFL_at_speed_bound'])),
                    }
        result['contact_samples'] = contact_samples
        if first_contact is not None:
            result['first_contact'] = first_contact
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
