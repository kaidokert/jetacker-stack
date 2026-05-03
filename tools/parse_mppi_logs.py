#!/usr/bin/env python3
"""Parse MPPI verbose logs from docker compose and tabulate second-by-second.

Usage: docker compose logs jetacker-controller-server | python tools/parse_mppi_logs.py T_START T_END
"""
import sys
import re
from collections import defaultdict

def main():
    t_start = float(sys.argv[1]) if len(sys.argv) > 1 else 0
    t_end = float(sys.argv[2]) if len(sys.argv) > 2 else 9e18

    # Parse all lines
    # Each MPPI cycle: 3 iterations of (B,C,D,D2), then E, F, G
    # We want per-cycle: the 3 C.costs lines and the G.cmd_vel

    cycles = []  # list of dicts
    current_cycle = None
    iter_in_cycle = 0

    for line in sys.stdin:
        if '[MPPI]' not in line:
            continue
        # Extract timestamp
        m = re.search(r'\[(\d+\.\d+)\]', line)
        if not m:
            continue
        t = float(m.group(1))
        if t < t_start or t > t_end:
            continue

        if '[MPPI] A.input' in line:
            # New cycle
            current_cycle = {'t': t, 'costs': [], 'cmd_vel': None, 'ess': [], 'best': []}
            iter_in_cycle = 0
        elif '[MPPI] C.costs' in line and current_cycle is not None:
            # C.costs  min=X max=Y mean=Z spread=W
            m2 = re.search(r'min=([\d.]+)\s+max=([\d.]+)\s+mean=([\d.]+)\s+spread=([\d.]+)', line)
            if m2:
                current_cycle['costs'].append({
                    'min': float(m2.group(1)),
                    'max': float(m2.group(2)),
                    'mean': float(m2.group(3)),
                    'spread': float(m2.group(4)),
                })
        elif '[MPPI] D.softmax' in line and current_cycle is not None:
            m2 = re.search(r'ess=(\d+)/(\d+)\s+best\[(\d+)\]:\s+cvx\(([-\d.]+),([-\d.]+),([-\d.]+)\)', line)
            if m2:
                current_cycle['ess'].append(int(m2.group(1)))
                current_cycle['best'].append({
                    'cvx0': float(m2.group(4)),
                    'cvx1': float(m2.group(5)),
                    'cvx2': float(m2.group(6)),
                })
        elif '[MPPI] G.cmd_vel' in line and current_cycle is not None:
            m2 = re.search(r'vx=([-\d.]+)\s+wz=([-\d.]+)', line)
            if m2:
                current_cycle['cmd_vel'] = (float(m2.group(1)), float(m2.group(2)))
            cycles.append(current_cycle)
            current_cycle = None

    if not cycles:
        print("No MPPI cycles found in time window")
        return

    t0 = cycles[0]['t']

    # Group by second
    by_second = defaultdict(list)
    for c in cycles:
        sec = int(c['t'] - t0)
        by_second[sec].append(c)

    # Print header
    print(f"{'t':>4s}  {'vx_avg':>7s} {'wz_avg':>7s}  "
          f"{'it1_min':>7s} {'it1_max':>7s} {'it1_spr':>7s}  "
          f"{'it3_min':>7s} {'it3_max':>7s} {'it3_spr':>7s}  "
          f"{'ess1':>4s} {'ess3':>4s}  "
          f"{'best_vx0':>8s}")
    print("-" * 110)

    for sec in sorted(by_second.keys()):
        group = by_second[sec]
        n = len(group)
        # Average cmd_vel
        vxs = [c['cmd_vel'][0] for c in group if c['cmd_vel']]
        wzs = [c['cmd_vel'][1] for c in group if c['cmd_vel']]
        vx_avg = sum(vxs) / len(vxs) if vxs else 0
        wz_avg = sum(wzs) / len(wzs) if wzs else 0

        # Average iteration costs (iter 0 = first, iter 2 = last/3rd)
        it1_mins, it1_maxs, it1_sprs = [], [], []
        it3_mins, it3_maxs, it3_sprs = [], [], []
        ess1s, ess3s = [], []
        best_vx0s = []

        for c in group:
            if len(c['costs']) >= 1:
                it1_mins.append(c['costs'][0]['min'])
                it1_maxs.append(c['costs'][0]['max'])
                it1_sprs.append(c['costs'][0]['spread'])
            if len(c['costs']) >= 3:
                it3_mins.append(c['costs'][2]['min'])
                it3_maxs.append(c['costs'][2]['max'])
                it3_sprs.append(c['costs'][2]['spread'])
            if len(c['ess']) >= 1:
                ess1s.append(c['ess'][0])
            if len(c['ess']) >= 3:
                ess3s.append(c['ess'][2])
            if len(c['best']) >= 3:
                best_vx0s.append(c['best'][2]['cvx0'])

        def avg(lst):
            return sum(lst) / len(lst) if lst else 0

        print(f"{sec:4d}  {vx_avg:7.4f} {wz_avg:7.4f}  "
              f"{avg(it1_mins):7.2f} {avg(it1_maxs):7.2f} {avg(it1_sprs):7.2f}  "
              f"{avg(it3_mins):7.2f} {avg(it3_maxs):7.2f} {avg(it3_sprs):7.2f}  "
              f"{avg(ess1s):4.0f} {avg(ess3s):4.0f}  "
              f"{avg(best_vx0s):8.4f}")

    print(f"\nTotal cycles: {len(cycles)}, seconds: {len(by_second)}")
    # Print per-cycle detail for the transition zone (seconds 25-30 from start)
    print(f"\n--- Per-cycle detail around stall onset (t=25-30s) ---")
    print(f"{'t_rel':>6s}  {'vx':>7s} {'wz':>7s}  {'it1_spr':>7s} {'it2_spr':>7s} {'it3_spr':>7s}  {'ess3':>4s} {'best_vx':>7s}")
    for c in cycles:
        t_rel = c['t'] - t0
        if 22 < t_rel < 35:
            vx = c['cmd_vel'][0] if c['cmd_vel'] else 0
            wz = c['cmd_vel'][1] if c['cmd_vel'] else 0
            sprs = [co['spread'] for co in c['costs']]
            ess3 = c['ess'][2] if len(c['ess']) >= 3 else 0
            best_vx = c['best'][2]['cvx0'] if len(c['best']) >= 3 else 0
            spr_str = ' '.join(f"{s:7.2f}" for s in sprs[:3])
            while len(sprs) < 3:
                sprs.append(0)
            print(f"{t_rel:6.1f}  {vx:7.4f} {wz:7.4f}  {sprs[0]:7.2f} {sprs[1]:7.2f} {sprs[2]:7.2f}  {ess3:4d} {best_vx:7.4f}")


if __name__ == '__main__':
    main()
