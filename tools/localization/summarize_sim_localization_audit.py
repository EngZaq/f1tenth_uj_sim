#!/usr/bin/env python3
"""Summarize passive simulator-only localization drift evidence.

Relative error is aligned to the first synced observation, so absolute
initial map registration cannot be assessed with this tool.
"""
import argparse
import csv
from pathlib import Path
from statistics import median


def stats(values):
    v = sorted(v for v in values if v == v and abs(v) < float('inf'))
    if not v:
        return "no synchronized samples"
    return (f"n={len(v)} median={median(v):.3f} "
            f"p95={v[min(len(v)-1,round(.95*(len(v)-1)))]:.3f} max={v[-1]:.3f}")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('csv_file',type=Path)
    ap.add_argument('--controller-csv',type=Path)
    a=ap.parse_args()
    with a.csv_file.open(newline='') as f:
        data=list(csv.DictReader(f))
    if not data:
        raise SystemExit("No synchronized truth/AMCL pairs found")
    print(f'PASSIVE TRUTH AUDIT: {len(data)} synchronized observations')
    for key in ['amcl_relative_pos_error_m','ekf_relative_pos_error_m',
                'amcl_relative_yaw_error_deg','ekf_relative_yaw_error_deg',
                'amcl_pos_std_m','amcl_yaw_std_deg','truth_pair_lag_s']:
        print(f'{key:34s} {stats([float(x[key]) for x in data])}')
    if a.controller_csv and a.controller_csv.exists():
        with a.controller_csv.open(newline='') as f:
            controller=list(csv.DictReader(f))
        for key in ['amcl_across_track_std_m','amcl_along_track_std_m','cte_m']:
            if controller and key in controller[0]:
                print(f'{key:34s} {stats([float(x[key]) for x in controller])}')
    print('Interpretation: relative trajectory drift vs first aligned pose. '
          'It does not establish absolute map registration accuracy.')


if __name__=='__main__':
    main()
