import argparse
import struct
import time
from collections import deque

import numpy as np

SYNC0 = 0xAA
SYNC1 = 0x55
TYPE_DATA = 0x01
TYPE_STATS = 0x02
N_POINTS = 360
SYNC_BYTES = bytes((SYNC0, SYNC1))

STATS_KEYS = (
    "frames_sent",
    "bytes_per_s",
    "packets",
    "invalid",
    "proc_us_max",
    "ht",
    "tc",
    "restarts",
    "uart_err",
    "drops",
    "imu_err",
)


def xor_checksum(ftype, length, payload):
    x = ftype ^ (length & 0xFF) ^ ((length >> 8) & 0xFF)
    for b in payload:
        x ^= b
    return x & 0xFF


def build_frame(ftype, payload):
    length = len(payload)
    head = bytes((SYNC0, SYNC1, ftype, length & 0xFF, (length >> 8) & 0xFF))
    return head + payload + bytes((xor_checksum(ftype, length, payload),))


class Telemetry:
    def __init__(self):
        self.buf = bytearray()
        self.scan_angles = np.arange(N_POINTS, dtype=np.float32)
        self.scan_dist = np.zeros(N_POINTS, dtype=np.float32)
        self.heading = 0.0
        self.gz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.frames = 0
        self.crc_errors = 0
        self.last_scan_time = 0.0
        self.stats = {}
        self.base = None

    @property
    def lost(self):
        sent = self.stats.get("frames_sent", 0)
        if self.base is None:
            return 0
        return max(0, (sent - self.frames) - self.base)

    def feed(self, data):
        self.buf.extend(data)
        while True:
            idx = self.buf.find(SYNC_BYTES)
            if idx < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return
            if idx:
                del self.buf[:idx]
            if len(self.buf) < 5:
                return
            length = self.buf[3] | (self.buf[4] << 8)
            total = 5 + length + 1
            if len(self.buf) < total:
                return
            ftype = self.buf[2]
            payload = bytes(self.buf[5:5 + length])
            crc = self.buf[5 + length]
            del self.buf[:total]
            if crc != xor_checksum(ftype, length, payload):
                self.crc_errors += 1
                continue
            self.frames += 1
            if ftype == TYPE_DATA:
                self._data(payload)
            elif ftype == TYPE_STATS:
                self._stats(payload)

    def _data(self, payload):
        if len(payload) < 10:
            return
        h, gz, ax, ay, n = struct.unpack_from("<hhhhH", payload, 0)
        if n != N_POINTS or len(payload) < 10 + 2 * n:
            return
        self.scan_dist = np.frombuffer(payload, dtype="<u2", count=n, offset=10).astype(np.float32)
        self.heading = h / 100.0
        self.gz = gz / 100.0
        self.ax = ax / 1000.0
        self.ay = ay / 1000.0
        self.last_scan_time = time.time()

    def _stats(self, payload):
        if len(payload) < 32:
            return
        self.stats = dict(zip(STATS_KEYS, struct.unpack_from("<IIIII6H", payload, 0)))
        if self.base is None:
            self.base = self.stats["frames_sent"] - self.frames


class Simulator:
    def __init__(self):
        self.t0 = time.time()
        self.next_t = self.t0
        self.angles = np.arange(N_POINTS, dtype=np.float32)
        self.count = 0

    def read(self):
        now = time.time()
        if now < self.next_t:
            return b""
        self.next_t = max(self.next_t + 0.1, now)
        t = now - self.t0
        heading = ((t * 60.0) % 360.0) - 180.0
        r = 2400.0 + 1300.0 * np.sin(np.deg2rad(2.0 * self.angles + t * 25.0))
        r = np.clip(r + np.random.normal(0.0, 25.0, N_POINTS), 250.0, 5500.0)
        r[(self.angles.astype(int) + int(t * 30)) % 53 == 0] = 0.0
        d = r.astype(np.uint16)
        payload = struct.pack("<hhhhH", int(heading * 100), 1234, 12, -8, N_POINTS)
        out = build_frame(TYPE_DATA, payload + d.astype("<u2").tobytes())
        self.count += 1
        if self.count % 10 == 0:
            stats = struct.pack(
                "<IIIII6H", self.count, 7360, 5000 * self.count, 3, 180, 240, 240, 0, 0, 0, 0
            )
            out += build_frame(TYPE_STATS, stats)
        return out


def run(port, baud, sim):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    tel = Telemetry()
    sim_src = Simulator() if sim else None
    ser = None
    if not sim:
        import serial

        ser = serial.Serial(port, baud, timeout=0)

    fig = plt.figure(figsize=(11.5, 6.2))
    fig.canvas.manager.set_window_title("CodingDay - RPLIDAR C1 + MPU6500")
    grid = fig.add_gridspec(1, 2, width_ratios=(1.25, 1.0))
    ax = fig.add_subplot(grid[0, 0])
    axp = fig.add_subplot(grid[0, 1], projection="polar")

    ax.set_aspect("equal")
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m) right")
    ax.set_ylabel("Y (m) forward")
    ax.set_title("2D scan")

    scat = ax.scatter([], [], s=14, c=[], cmap="viridis", vmin=0, vmax=6000)
    fig.colorbar(scat, ax=ax, label="distance (mm)")

    axp.set_theta_zero_location("N")
    axp.set_theta_direction(-1)
    axp.set_ylim(0, 1)
    axp.set_yticks([])
    axp.set_title("heading")
    head_line, = axp.plot([], [], color="tab:red", lw=5)
    head_dot = axp.scatter([], [], color="tab:red", s=50)
    head_txt = axp.text(0, 0.5, "", ha="center", va="center", fontsize=12, color="tab:red")

    status = fig.text(0.01, 0.01, "", family="monospace", fontsize=8.5, va="bottom")
    fps_hist = deque(maxlen=40)

    def update(_):
        data = sim_src.read() if sim else ser.read(4096)
        if data:
            tel.feed(data)

        theta = np.deg2rad(tel.scan_angles)
        dist_m = tel.scan_dist / 1000.0
        mask = dist_m > 0.05
        scat.set_offsets(
            np.column_stack((dist_m[mask] * np.sin(theta[mask]), dist_m[mask] * np.cos(theta[mask])))
        )
        scat.set_array(tel.scan_dist[mask])

        rad = np.deg2rad(tel.heading)
        head_line.set_data([0.0, rad], [0.0, 1.0])
        head_dot.set_offsets([[rad, 1.0]])
        head_txt.set_text(f"{tel.heading:+.1f} deg")

        now = time.time()
        fps_hist.append(now)
        fps = 0.0
        if len(fps_hist) > 1:
            span = fps_hist[-1] - fps_hist[0]
            if span > 0:
                fps = (len(fps_hist) - 1) / span

        st = tel.stats
        sent = st.get("frames_sent", 0)
        lost = tel.lost
        loss = (100.0 * lost / sent) if sent else 0.0
        status.set_text(
            f"fps {fps:4.1f}  frames {tel.frames}  crc_err {tel.crc_errors}\n"
            f"MCU sent {sent}  lost {lost} ({loss:.1f}%)\n"
            f"LiDAR {st.get('bytes_per_s', 0)} B/s  pkt {st.get('packets', 0)}  "
            f"inv {st.get('invalid', 0)}  max {st.get('proc_us_max', 0)} us\n"
            f"IMU gz {tel.gz:+.1f} dps  ax {tel.ax:+.2f} g  ay {tel.ay:+.2f} g  "
            f"err {st.get('imu_err', 0)}\n"
            f"BT drops {st.get('drops', 0)}  UART err {st.get('uart_err', 0)}  "
            f"restarts {st.get('restarts', 0)}"
        )
        return scat, head_line, head_dot, head_txt, status

    anim = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    fig._anim = anim
    fig.subplots_adjust(left=0.07, right=0.97, top=0.93, bottom=0.20)
    plt.show()

    if ser is not None:
        ser.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="HC-04 Bluetooth COM port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--sim", action="store_true", help="no hardware, synthetic data")
    args = parser.parse_args()

    if not args.sim and not args.port:
        import serial.tools.list_ports

        for p in serial.tools.list_ports.comports():
            print(f"{p.device:8s} {p.description}")
        print("usage: python main.py --port COMx --baud 230400")
        return

    run(args.port, args.baud, args.sim)


if __name__ == "__main__":
    main()
