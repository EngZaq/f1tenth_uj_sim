#!/usr/bin/env python3
"""Summarize an existing profiled-run controller log, with optional truth audit.

Older runs predate localization_truth_audit.csv. They can report AMCL's
covariance and tracking error but cannot measure localization truth drift.
Truth audit error is relative to the first aligned observation.
Motion analysis uses recorded truth displacement, never commanded speed.
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
    rms = math.sqrt(sum(x*x for x in values) / len(values))
    return (f'n={len(values)} rms={rms:.3f} median={median(values):.3f} '
            f'p95={values[min(len(values)-1, round(.95*(len(values)-1)))]:.3f} '
            f'max={values[-1]:.3f}')


def number(row, name):
    try:
        value = float(row.get(name, 'nan'))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def sampled_truth_span(rows):
    """Describe observed endpoints without inventing motion between samples."""
    points = []
    for row in rows:
        values = [number(row, name) for name in ('stamp_s','truth_x_sim','truth_y_sim')]
        if all(v is not None for v in values):
            points.append(values)
    if not points:
        return None
    first = points[0]
    return {'samples': len(points), 'span_s': max(p[0] for p in points)-min(p[0] for p in points),
            'max_displacement_m': max(math.hypot(p[1]-first[1], p[2]-first[2]) for p in points)}


def vehicle_covariance_sigma(row):
    """Project valid XY covariance on estimated vehicle axes, not route axes."""
    values = [number(row, name) for name in
              ('amcl_cov_xx','amcl_cov_xy','amcl_cov_yx','amcl_cov_yy','amcl_yaw_deg')]
    if any(v is None for v in values):
        return None
    xx, xy, yx, yy, heading = values
    if xx < 0 or yy < 0 or abs(xy-yx) > 1e-9:
        return None
    cross = .5*(xy+yx)
    eigen_min = .5*(xx+yy-math.hypot(xx-yy, 2*cross))
    if eigen_min < -1e-12:
        return None
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    forward = c*c*xx+2*c*s*cross+s*s*yy
    left = s*s*xx-2*c*s*cross+c*c*yy
    return math.sqrt(max(0.,forward)), math.sqrt(max(0.,left))


def motion_audit(rows, max_pair_lag=0.05, motion_speed=0.05, max_gap=0.5):
    """Classify interval endpoints using truth XY / recorded AMCL stamp.

    Missing/late data and clock resets break intervals. Never sort a log or
    bridge a rejected sample. Signed components use truth vehicle heading,
    NOT a route normal. Initial alignment reproduces the passive recorder;
    it removes initial pose error and cannot measure absolute map accuracy.
    """
    groups = {'MOVING': [], 'STATIONARY': []}
    counts = {'invalid': 0, 'late_pair': 0, 'nonmonotonic': 0,
              'gap': 0, 'no_interval': 0}
    duration = dict.fromkeys(groups, 0.0)
    distance = dict.fromkeys(groups, 0.0)
    required = ('stamp_s', 'truth_x_sim', 'truth_y_sim', 'truth_pair_lag_s')
    anchor_fields = ('truth_x_sim', 'truth_y_sim', 'truth_yaw_deg',
                     'amcl_x_map', 'amcl_y_map', 'amcl_yaw_deg')
    anchor = [number(rows[0], key) for key in anchor_fields] if rows else []
    transform = None
    if anchor and all(x is not None for x in anchor):
        tx, ty, th, ax, ay, ah = anchor
        rotation = math.radians(ah-th)
        c, s = math.cos(rotation), math.sin(rotation)
        transform = (c, s, ax-c*tx+s*ty, ay-s*tx-c*ty, rotation)
    previous = None
    last_timestamp = None
    for row in rows:
        values = [number(row, key) for key in required]
        if any(x is None for x in values):
            counts['invalid'] += 1
            previous = None
            continue
        t, x, y, lag = values
        # New recordings retain exact source stamps; old recordings only
        # provide a rounded absolute pairing lag and the AMCL timestamp.
        motion_t = number(row, 'truth_stamp_s') if 'truth_stamp_s' in row else t
        if motion_t is None:
            counts['invalid'] += 1
            previous = None
            continue
        if 'truth_stamp_s' in row:
            lag = abs(motion_t-t)
        if last_timestamp is not None and t <= last_timestamp:
            counts['nonmonotonic'] += 1
            # Start a new segment after a reset; never evaluate its jump.
            last_timestamp = t
            previous = None
            continue
        last_timestamp = t
        if not 0 <= lag <= max_pair_lag:
            counts['late_pair'] += 1
            previous = None
            continue
        current = (motion_t, x, y)
        if previous is None:
            counts['no_interval'] += 1
            previous = current
            continue
        dt = motion_t-previous[0]
        step = math.hypot(x-previous[1], y-previous[2])
        previous = current
        if dt <= 0:
            counts['nonmonotonic'] += 1
            previous = None
            continue
        if dt > max_gap:
            counts['gap'] += 1
            continue
        speed = step/dt
        label = 'MOVING' if speed >= motion_speed else 'STATIONARY'
        sample = dict(row)
        sample['interval_truth_speed_mps'] = speed
        if transform is not None:
            ax, ay, heading = (number(row, k) for k in
                               ('amcl_x_map', 'amcl_y_map', 'truth_yaw_deg'))
            if ax is not None and ay is not None and heading is not None:
                c, s, ox, oy, rotation = transform
                dx, dy = ax-(c*x-s*y+ox), ay-(s*x+c*y+oy)
                h = math.radians(heading)+rotation
                sample['amcl_relative_vehicle_forward_error_m'] = (
                    math.cos(h)*dx+math.sin(h)*dy)
                sample['amcl_relative_vehicle_left_error_m'] = (
                    -math.sin(h)*dx+math.cos(h)*dy)
        groups[label].append(sample)
        duration[label] += dt
        distance[label] += step
    return groups, counts, duration, distance


def print_motion_audit(rows, args):
    groups, counts, duration, distance = motion_audit(
        rows, args.max_pair_lag_s, args.motion_threshold_mps, args.max_sample_gap_s)
    print('\nMOTION-SEPARATED TRUTH AUDIT (interval endpoints):')
    print(f'Pair lag <= {args.max_pair_lag_s:.3f}s; '
          f'moving >= {args.motion_threshold_mps:.3f}m/s; '
          f'max interval {args.max_sample_gap_s:.3f}s')
    print('Excluded/unclassified rows:', counts)
    span = sampled_truth_span(rows)
    if span:
        print(f'OBSERVED TRUTH ENDPOINTS: n={span["samples"]} '
              f'span={span["span_s"]:.3f}s '
              f'max displacement from first={span["max_displacement_m"]:.5f}m')
        print('Endpoint agreement does not prove zero motion between sparse samples.')
    projection = vehicle_covariance_sigma(rows[-1])
    if projection:
        print(f'Last covariance in ESTIMATED VEHICLE axes: forward_sigma={projection[0]:.4f}m '
              f'left_sigma={projection[1]:.4f}m (not a route-clearance verdict)')
    for label, samples in groups.items():
        print(f'{label}: n={len(samples)} interval_time={duration[label]:.3f}s '
              f'truth_distance={distance[label]:.3f}m')
        for field in ('amcl_relative_pos_error_m', 'amcl_relative_yaw_error_deg',
                      'ekf_relative_pos_error_m', 'amcl_pos_std_m',
                      'amcl_yaw_std_deg', 'interval_truth_speed_mps',
                      'amcl_relative_vehicle_forward_error_m',
                      'amcl_relative_vehicle_left_error_m'):
            values = finite_values(samples, field)
            if values:
                print(f'  {field:45s} {stats(values)}')
    if not groups['MOVING']:
        print('NO ACCEPTED MOVING EVIDENCE. Stationary data cannot validate driving.')
    if counts['nonmonotonic']:
        print('WARNING: duplicate/reset timestamps found; inspect run boundaries '
              'and frame continuity before interpreting relative drift.')
    anchor_lag = number(rows[0], 'truth_pair_lag_s')
    if anchor_lag is None or not 0 <= anchor_lag <= args.max_pair_lag_s:
        print('WARNING: initial alignment pair fails this timing filter; '
              'the recorded relative errors still inherit that alignment.')
    if 'truth_stamp_s' in rows[0]:
        print('Motion speed is an interval average using recorded truth sample stamps.')
    else:
        print('Motion speed is an interval average from recorded truth positions. '
              'Exact truth sample stamps were not saved; timing bias can remain.')
    print('Forward/left errors use the TRUE VEHICLE axes, not route tangent/normal.')
    if 'amcl_cov_xy' in rows[0]:
        print('Full planar covariance is recorded for follow-up analysis; this summary '
              'provides no covariance-consistency or clearance-safety verdict.')
    else:
        print('Full XY covariance was not saved: no covariance-consistency or '
              'clearance-safety verdict is possible from this CSV alone.')


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
    p.add_argument('--max-pair-lag-s', type=float, default=0.05,
                   help='Timing filter for motion-only diagnostics (default: 0.05s)')
    p.add_argument('--motion-threshold-mps', type=float, default=0.05,
                   help='Truth interval-average speed classified moving (default: 0.05m/s)')
    p.add_argument('--max-sample-gap-s', type=float, default=0.5,
                   help='Do not classify motion across longer gaps (default: 0.5s)')
    args = p.parse_args()
    for name in ('max_pair_lag_s', 'motion_threshold_mps', 'max_sample_gap_s'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            p.error(f'--{name.replace("_", "-")} must be finite and positive')
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
        else:
            indices = finite_values(controller, 'waypoint_index')
            if indices:
                print('Maximum waypoint index (not lap progress):', int(max(indices)))
        laps = finite_values(controller, 'lap')
        if laps:
            print('Maximum completed laps:', int(max(laps)))
        for name in ('cmd_speed_mps', 'cte_m', 'amcl_age_s',
                     'predicted_body_gap_m', 'amcl_pos_std_m',
                     'amcl_yaw_std_deg', 'amcl_across_track_std_m',
                     'amcl_along_track_std_m'):
            aliases = {'cmd_speed_mps': 'commanded_speed_mps',
                       'cte_m': 'estimated_cte_m'}
            column = name if name in controller[0] else aliases.get(name, name)
            if column in controller[0]:
                print(f'{column:35s} {stats(finite_values(controller, column))}')
        stops = [row for row in controller
                 if ('SAFETY HOLD' in row.get('reason', '') or
                     'LOCALIZATION PAUSE' in row.get('reason', ''))]
        if stops:
            print('First safety/pause reason:', stops[0]['reason'])
        drove = False
        for row in controller:
            if row.get('state') == 'DRIVE':
                drove = True
            elif drove and row.get('state') == 'STOP':
                print('First STOP after DRIVE:', row.get('reason', 'unknown'))
                break
        print('Last recorded state:', controller[-1].get('state'),
              '| reason:', controller[-1].get('reason', ''))

    if truth:
        print(f'\nPASSIVE TRUTH AUDIT: {len(truth)} recorded paired observations '
              '(all samples, unfiltered)')
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
        print_motion_audit(truth, args)
    else:
        print('\nNO TRUTH AUDIT IN THIS RUN. This is expected for older logs.')
        print('Controller CTE and AMCL covariance are estimates; neither '
              'proves actual localization accuracy.')


if __name__ == '__main__':
    main()
