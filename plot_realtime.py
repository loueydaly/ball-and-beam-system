#!/usr/bin/env python3
"""
Real-time plotter for the STM32 adaptive SMC project.
Reads lines from serial in the format produced by the firmware:
  e:<int> s:<int> v:<int> a^:<int> u:<int> steps:<int> de:<int>\r\n
Scales in firmware: e,s,v,de -> value*1000; a_hat -> value*100000; u -> value*1000

Usage:
  python plot_realtime.py --port COM3 --baud 115200

Requires: pyserial, matplotlib
  pip install pyserial matplotlib
"""

import argparse
import re
import sys
from collections import deque
import threading
import time

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

LINE_RE = re.compile(r"e:(-?\d+)\s+s:(-?\d+)\s+v:(-?\d+)\s+a\^:(-?\d+)\s+u:(-?\d+)\s+steps:(-?\d+)\s+de:(-?\d+)")

class SerialReader(threading.Thread):
    def __init__(self, port, baud, maxlen=1000):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()
        self.running = True
        self.data = {
            't': deque(maxlen=maxlen),
            'e': deque(maxlen=maxlen),
            'u': deque(maxlen=maxlen),
            'a': deque(maxlen=maxlen),
            'steps': deque(maxlen=maxlen),
        }
        self.start_time = time.time()

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            m = LINE_RE.search(line)
            if m:
                e = int(m.group(1)) / 1000.0
                s = int(m.group(2)) / 1000.0
                v = int(m.group(3)) / 1000.0
                a = int(m.group(4)) / 100000.0
                u = int(m.group(5)) / 1000.0
                steps = int(m.group(6))
                de = int(m.group(7)) / 1000.0
                t = time.time() - self.start_time
                with self.lock:
                    self.data['t'].append(t)
                    self.data['e'].append(e)
                    self.data['u'].append(u)
                    self.data['a'].append(a)
                    self.data['steps'].append(steps)

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def get_data(self):
        with self.lock:
            return {k: list(v) for k,v in self.data.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM3', help='Serial port (e.g., COM3 or /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--window', type=int, default=10, help='Seconds of data to show')
    args = parser.parse_args()

    reader = SerialReader(args.port, args.baud, maxlen=10000)
    reader.start()

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ln_e, = ax1.plot([], [], label='error (m)')
    ln_u, = ax2.plot([], [], 'r', label='u (rad)')
    ln_a, = ax2.plot([], [], 'g', label='a_hat')

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Error (m)')
    ax2.set_ylabel('Control / a_hat')

    ax1.legend(loc='upper left')
    ax2.legend(loc='upper right')

    def init():
        ax1.set_xlim(0, args.window)
        ax1.set_ylim(-1.0, 1.0)
        ax2.set_ylim(-10.0, 10.0)
        return ln_e, ln_u, ln_a

    def update(frame):
        d = reader.get_data()
        if not d['t']:
            return ln_e, ln_u, ln_a
        t0 = d['t'][-1] - args.window
        t = [tt - t0 for tt in d['t']]
        ln_e.set_data(t, d['e'])
        ln_u.set_data(t, d['u'])
        ln_a.set_data(t, d['a'])

        ax1.set_xlim(0, args.window)
        # autoscale y for error
        if d['e']:
            emin = min(d['e'])
            emax = max(d['e'])
            margin = max(0.001, (emax - emin) * 0.2)
            ax1.set_ylim(emin - margin, emax + margin)
        return ln_e, ln_u, ln_a

    ani = animation.FuncAnimation(fig, update, init_func=init, interval=100, blit=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()

if __name__ == '__main__':
    main()
