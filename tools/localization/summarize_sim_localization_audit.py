#!/usr/bin/env python3
"""Summarize an existing profiled-run controller log, with optional truth audit.

Older runs predate localization_truth_audit.csv. They can report AMCL's
covariance and tracking error but cannot measure localization truth drift.
Truth audit error is relative to the first aligned observation.
"""
import argparse
import csv
import math
from pathlib import Path
from statistics import median


def finite_values(rows, field):
    result = []
    for row in rows:
        try:
            number = float(row.get(field, 'nan'))
            if math.isfinite(number):
                result.append(number)
        except (TypeError, ValueError):
            pass
    return result


def stats(values):
    if not values:
        return 'no recorded values'
    values = sorted(values)
    return (f'n={len(values)} median={median(values):.3f} '
            f'p95={values[min(len(values)-1, round(.95*(len(values)-1)))]:.3f} '
            f'max={values[-1]:.3f}')


def read_csv(path):
    if path is None or not path.is_file():
        return []
    with path.open(newline='') as fp:
        return list(csv.DictReader(fp))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv_file', nargs='?', type=Path,
                   help='Optional localization_truth_audit.csv (new runs only)')
    p.add_argument('--controller-csv', type=Path,
                   help='estimated_drive.csv, available on older runs')
    args = p.parse_args()
    truth = read_csv(args.csv_file)
    controller = read_csv(args.controller_csv)

    if not truth and not controller:
        p.error('No readable CSV found. For an older run, pass '
                '--controller-csv path/to/estimated_drive.csv')

    if controller:
        print(f'CONTROLLER CSV: {args.controller_csv} ({len(controller)} rows)')
        states = {}
        for row in controller:
            name = row.get('state', 'unknown')
            states[name] = states.get(name, 0) + 1
        print('State counts:', states)
        progress = finite_values(controller, 'progress')
        if progress:
            print('Maximum progress:', int(max(progress)), 'waypoints')
        laps = finite_values(controller, 'lap')
        if laps:
            print('Maximum completed laps:', int(max(laps)))
        for name in ('cmd_speed_mps', 'cte_m', 'amcl_age_s',
                     'predicted_body_gap_m', 'amcl_pos_std_m',
                     'amcl_yaw_std_deg', 'amcl_across_track_std_m',
                     'amcl_along_track_std_m'):
            if name in controller[0]:
                print(f'{name:35s} {stats(finite_values(controller, name))}')
        stops = [row for row in controller
                 if ('SAFETY HOLD' in row.get('reason', '') or
                     'LOCALIZATION PAUSE' in row.get('reason', ''))]
        if stops:
            print('First safety/pause reason:', stops[0]['reason'])
        print('Last recorded state:', controller[-1].get('state'),
              '| reason:', controller[-1].get('reason', ''))

    if truth:
        print(f'\nPASSIVE TRUTH AUDIT: {len(truth)} synchronized observations')
        for name in ('amcl_relative_pos_error_m',
                     'ekf_relative_pos_error_m',
                     'amcl_relative_yaw_error_deg',
                     'ekf_relative_yaw_error_deg',
                     'amcl_pos_std_m', 'amcl_yaw_std_deg',
                     'truth_pair_lag_s', 'ekf_pair_lag_s'):
            if name in truth[0]:
                print(f'{name:35s} {stats(finite_values(truth, name))}')
        print('Errors are relative to first truth/estimate alignment, '
              'not absolute map registration accuracy.')
    else:
        print('\nNO TRUTH AUDIT IN THIS RUN. This is expected for older logs.')
        print('Controller CTE and AMCL covariance are estimates; neither '
              'proves actual localization accuracy.')


if __name__ == '__main__':
    main()
