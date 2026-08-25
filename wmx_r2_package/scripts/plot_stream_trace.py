#!/usr/bin/env python3
# Copyright 2026 Movensys Corporation.
# Licensed under the MIT License. See LICENSE.txt for details.
#
# plot_stream_trace
#
# Plots a CSV from capture_stream_trace.py and prints the tracking metrics.
#
# The point of the three-trace form is that it SPLITS the error, which the
# earlier setpoint-vs-encoder measurements could not:
#
#   setpoint -> pos_cmd      the API buffer and interpolation path
#   pos_cmd  -> actual_pos   the servo loop underneath it
#
# A residual that lives in the first split is this node's problem; one that
# lives in the second is tuning, and no amount of work on the streaming path
# will move it.
#
# Usage:
#   python3 plot_stream_trace.py trace.csv                 # joint 1
#   python3 plot_stream_trace.py trace.csv --joint 3
#   python3 plot_stream_trace.py trace.csv --all           # 6-up overview
#   python3 plot_stream_trace.py trace.csv --show          # interactive window

import argparse
import csv
from collections import defaultdict

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# Same slots as the published trace artifact, so an entity keeps its colour
# across both: blue = asked for, aqua = what WMX3 generated, orange = what moved.
C_SET, C_CMD, C_ACT = '#2a78d6', '#1baf7a', '#eb6834'
C_GRID, C_AXIS, C_MUTE = '#e1e0d9', '#c3c2b7', '#898781'


def load(path):
    series = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            n = sum(1 for k in row if k.startswith('j'))
            series[row['source']].append(
                [float(row['t'])] + [float(row[f'j{i + 1}']) for i in range(n)])
    out = {}
    for k, v in series.items():
        a = np.array(sorted(v, key=lambda r: r[0]))
        out[k] = (a[:, 0], a[:, 1:])
    return out


def align(t_ref, t_src, y_src):
    """Linear-interpolate y_src onto t_ref."""
    return np.interp(t_ref, t_src, y_src)


def lag_gain(ref, sig, dt):
    """Cross-correlate to find delay, then fit gain and residual at that delay."""
    r = ref - ref.mean()
    s = sig - sig.mean()
    best, best_score = 0, -np.inf
    for lag in range(0, int(0.6 / dt)):
        m = len(r) - lag
        if m < 10:
            break
        score = float(np.dot(r[:m], s[lag:lag + m])) / m
        if score > best_score:
            best, best_score = lag, score
    m = len(r) - best
    gain = float(np.dot(r[:m], s[best:best + m]) / np.dot(r[:m], r[:m]))
    resid = s[best:best + m] - gain * r[:m]
    return best * dt, gain, float(np.sqrt((resid ** 2).mean()))


def analyse(data, j):
    t_state, _ = data['pos_cmd']
    t = t_state - t_state[0]
    dt = float(np.median(np.diff(t)))
    cmd = data['pos_cmd'][1][:, j]
    act = data['actual_pos'][1][:, j]
    st_t, st_y = data['setpoint'][0] - t_state[0], data['setpoint'][1][:, j]
    setp = align(t, st_t, st_y)

    m = {}
    m['buffer'] = lag_gain(setp, cmd, dt)          # setpoint -> pos_cmd
    m['servo'] = lag_gain(cmd, act, dt)            # pos_cmd  -> actual_pos
    m['total'] = lag_gain(setp, act, dt)           # setpoint -> actual_pos

    v_cmd = np.gradient(cmd, t)
    v_act = np.gradient(act, t)
    span = act.max() - act.min()
    m['peak_v'] = float(np.abs(v_act).max())
    m['flips'] = int(np.sum(np.diff(np.sign(v_act)) != 0))
    m['dur'] = float(t[-1] - t[0])
    m['amp'] = span / 2
    return t, setp, cmd, act, v_cmd, v_act, m


def style(ax, ylabel):
    ax.grid(True, color=C_GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(C_AXIS)
    ax.tick_params(colors=C_MUTE, labelsize=9)
    ax.set_ylabel(ylabel, fontsize=9, color=C_MUTE)


def plot_joint(data, j, path, show):
    t, setp, cmd, act, v_cmd, v_act, m = analyse(data, j)
    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True,
                           gridspec_kw={'height_ratios': [3, 2, 2]})
    fig.suptitle(f'joint{j + 1} — commanded position vs motion executed by WMX3',
                 fontsize=13, x=0.02, ha='left')

    ax[0].plot(t, setp, color=C_SET, lw=1.6, label='setpoint (ROS)')
    ax[0].plot(t, cmd, color=C_CMD, lw=1.6, label='pos_cmd (WMX3 interpolator)')
    ax[0].plot(t, act, color=C_ACT, lw=1.6, label='actual_pos (encoder)')
    style(ax[0], 'position [rad]')
    ax[0].legend(frameon=False, fontsize=9, loc='upper right', ncol=3)

    ax[1].plot(t, (cmd - setp) * 1000, color=C_CMD, lw=1.4,
               label=f'buffer path: setpoint → pos_cmd  ({m["buffer"][0] * 1000:.0f} ms)')
    ax[1].plot(t, (act - cmd) * 1000, color=C_ACT, lw=1.4,
               label=f'servo loop: pos_cmd → actual  ({m["servo"][0] * 1000:.0f} ms)')
    ax[1].axhline(0, color=C_AXIS, lw=1)
    style(ax[1], 'error [mrad]')
    ax[1].legend(frameon=False, fontsize=9, loc='upper right')

    ax[2].plot(t, v_cmd, color=C_CMD, lw=1.4, label='pos_cmd velocity')
    ax[2].plot(t, v_act, color=C_ACT, lw=1.4, label='encoder velocity')
    ax[2].axhline(0, color=C_AXIS, lw=1)
    style(ax[2], 'velocity [rad/s]')
    ax[2].set_xlabel('time [s]', fontsize=9, color=C_MUTE)
    ax[2].legend(frameon=False, fontsize=9, loc='upper right', ncol=2)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(path, dpi=140)
    print(f'wrote {path}')
    if show:
        plt.show()


def plot_all(data, path, show):
    n = data['pos_cmd'][1].shape[1]
    fig, axes = plt.subplots(n, 1, figsize=(11, 2.0 * n), sharex=True)
    fig.suptitle('All joints — setpoint vs pos_cmd vs actual_pos',
                 fontsize=13, x=0.02, ha='left')
    for j in range(n):
        t, setp, cmd, act, _, _, _ = analyse(data, j)
        ax = axes[j]
        ax.plot(t, setp, color=C_SET, lw=1.2)
        ax.plot(t, cmd, color=C_CMD, lw=1.2)
        ax.plot(t, act, color=C_ACT, lw=1.2)
        style(ax, f'j{j + 1} [rad]')
    axes[-1].set_xlabel('time [s]', fontsize=9, color=C_MUTE)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(path, dpi=140)
    print(f'wrote {path}')
    if show:
        plt.show()


def report(data):
    n = data['pos_cmd'][1].shape[1]
    print(f'\n{"joint":>6} {"buf lag":>8} {"servo lag":>10} {"tot lag":>8} '
          f'{"gain":>7} {"buf rms":>9} {"servo rms":>10} {"peak v":>8} {"flips/s":>8}')
    for j in range(n):
        _, _, _, _, _, _, m = analyse(data, j)
        print(f'{"j" + str(j + 1):>6} '
              f'{m["buffer"][0] * 1000:7.0f}m {m["servo"][0] * 1000:9.0f}m '
              f'{m["total"][0] * 1000:7.0f}m {m["total"][1]:7.4f} '
              f'{m["buffer"][2] * 1000:8.3f}m {m["servo"][2] * 1000:9.3f}m '
              f'{m["peak_v"]:8.4f} {m["flips"] / m["dur"]:8.2f}')
    print('\nlag in ms, rms in mrad, peak v in rad/s')
    print('buf = setpoint -> pos_cmd (streaming path)   '
          'servo = pos_cmd -> actual (tuning)')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('--joint', type=int, default=1, help='1-based')
    p.add_argument('--all', action='store_true', help='6-up position overview')
    p.add_argument('--out', default=None)
    p.add_argument('--show', action='store_true')
    a = p.parse_args()

    if not a.show:
        matplotlib.use('Agg')

    data = load(a.csv)
    missing = {'setpoint', 'pos_cmd', 'actual_pos'} - set(data)
    if missing:
        raise SystemExit(
            f'{a.csv} has no rows for: {", ".join(sorted(missing))}. '
            'Was the command source publishing while it was captured?')

    if a.all:
        plot_all(data, a.out or 'stream_trace_all.png', a.show)
    else:
        plot_joint(data, a.joint - 1,
                   a.out or f'stream_trace_j{a.joint}.png', a.show)
    report(data)


if __name__ == '__main__':
    main()
