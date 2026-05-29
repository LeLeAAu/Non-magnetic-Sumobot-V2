import serial
import serial.tools.list_ports
import threading
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox

# Định nghĩa cấu trúc packet 
TELEMETRY_TYPE = 0x01
PARAM_SET_TYPE = 0x02
PARAM_GET_TYPE = 0x03

# Mapping tham số (cùng ID với firmware)
PARAM_IDS = {
    'WARN_DIST': 1,
    'STRIKE_DIST': 2,
    'PWM_MAX': 3,
    'PWM_STRIKE_HOLD': 4,
    'PWM_HIGH': 5,
    'PWM_MED': 6,
    'PWM_LOW': 7,
    'V_MAX_60': 8,
    'OMEGA_60': 9,
    'KP_STEERING': 10,
    'FEINT_CHANCE': 11,
    'ATK_LOCK_TIME': 12,
}

# Tên tham số (hiển thị)
PARAM_NAMES = {
    1: 'WARN_DIST (mm)',
    2: 'STRIKE_DIST (mm)',
    3: 'PWM_MAX',
    4: 'PWM_STRIKE_HOLD',
    5: 'PWM_HIGH',
    6: 'PWM_MED',
    7: 'PWM_LOW',
    8: 'V_MAX_60 (mm/s)',
    9: 'OMEGA_60 (deg/s)',
    10: 'KP_STEERING',
    11: 'FEINT_CHANCE (%)',
    12: 'ATK_LOCK_TIME (ms)',
}

# CRC16-CCITT
def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# COBS decode
def cobs_decode(data):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        for j in range(code - 1):
            if i >= len(data):
                return None
            out.append(data[i])
            i += 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return out

# Giao diện chính
class RobotTelemetryUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Sumo Telemetry & Tuning")
        self.serial_port = None
        self.running = False
        self.telemetry_data = {
            'timestamp': [], 'dist0': [], 'enemy_angle': [], 'v_e': [], 'state': []
        }
        self.max_points = 200

        # Tạo notebook (tab)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True)

        # Tab 1: Đồ thị
        self.plot_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.plot_frame, text="Biểu đồ")

        # Tab 2: Điều khiển tham số
        self.param_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.param_frame, text="Tuning")

        # Tab 3: Kết nối Serial
        self.serial_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.serial_frame, text="Kết nối")

        self.setup_serial_tab()
        self.setup_plot_tab()
        self.setup_param_tab()

    #Tab Serial
    def setup_serial_tab(self):
        ttk.Label(self.serial_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5)
        self.combo_port = ttk.Combobox(self.serial_frame, values=self.get_serial_ports())
        self.combo_port.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(self.serial_frame, text="Kết nối", command=self.connect_serial).grid(row=0, column=2, padx=5)
        ttk.Button(self.serial_frame, text="Ngắt kết nối", command=self.disconnect_serial).grid(row=0, column=3, padx=5)
        self.status_label = ttk.Label(self.serial_frame, text="Chưa kết nối", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=4, pady=10)

        # Khung hiển thị trạng thái hiện tại
        self.state_text = tk.StringVar(value="State: --")
        ttk.Label(self.serial_frame, textvariable=self.state_text, font=('Arial', 12)).grid(row=2, column=0, columnspan=4)

    def get_serial_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect_serial(self):
        port = self.combo_port.get()
        if not port:
            messagebox.showerror("Lỗi", "Chọn cổng COM")
            return
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.running = True
            self.status_label.config(text=f"Đã kết nối {port}", foreground="green")
            # Yêu cầu ESP gửi lại toàn bộ thông số
            self.send_param_get()
            # Bắt đầu luồng đọc
            self.read_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể mở cổng {port}\n{e}")

    def disconnect_serial(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_label.config(text="Đã ngắt kết nối", foreground="red")
        self.state_text.set("State: --")

    def send_param_get(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        # Tạo gói PARAM_GET (type=0x03, len=0)
        frame = bytearray([PARAM_GET_TYPE, 0, 0])  # type + length(0)
        crc = crc16_ccitt(frame)
        frame += struct.pack('<H', crc)
        # COBS encode
        cobs_frame = self.cobs_encode(frame)
        self.serial_port.write(b'\x00' + cobs_frame + b'\x00')

    def send_param_set(self, param_id, value):
        if not self.serial_port or not self.serial_port.is_open:
            return
        # Giá trị gửi luôn là float 4 byte
        val_bytes = struct.pack('<f', float(value))
        payload = struct.pack('<H', param_id) + val_bytes
        frame = bytearray([PARAM_SET_TYPE, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
        crc = crc16_ccitt(frame)
        frame += struct.pack('<H', crc)
        cobs_frame = self.cobs_encode(frame)
        self.serial_port.write(b'\x00' + cobs_frame + b'\x00')

    @staticmethod
    def cobs_encode(data):
        out = bytearray()
        out.append(0x01)  # code bắt đầu
        code_idx = 0
        code = 1
        for b in data:
            if b == 0:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
            else:
                out.append(b)
                code += 1
                if code == 0xFF:
                    out[code_idx] = code
                    code = 1
                    code_idx = len(out)
                    out.append(0)
        out[code_idx] = code
        return out

    def serial_reader(self):
        rx_buf = bytearray()
        in_packet = False
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    data = self.serial_port.read(1024)
                    for byte in data:
                        if not in_packet and byte == 0x00:
                            in_packet = True
                            rx_buf = bytearray()
                        elif in_packet and byte == 0x00:
                            # Kết thúc packet
                            if len(rx_buf) > 0:
                                decoded = cobs_decode(rx_buf)
                                if decoded and len(decoded) >= 3:
                                    msg_type = decoded[0]
                                    pkt_len = decoded[1] | (decoded[2] << 8)
                                    if len(decoded) >= 3 + pkt_len + 2:
                                        recv_crc = struct.unpack('<H', decoded[3+pkt_len:3+pkt_len+2])[0]
                                        calc_crc = crc16_ccitt(decoded[:3+pkt_len])
                                        if recv_crc == calc_crc:
                                            if msg_type == TELEMETRY_TYPE:
                                                self.process_telemetry(decoded[3:3+pkt_len])
                                            elif msg_type == 0x04:  # PARAM_RESP
                                                self.process_param_response(decoded[3:3+pkt_len])
                            in_packet = False
                        elif in_packet:
                            rx_buf.append(byte)
                except Exception as e:
                    print("Serial read error:", e)
            time.sleep(0.001)

    def process_telemetry(self, payload):
        # Cấu trúc giống firmware (chú ý little-endian)
        if len(payload) < 46:
            return
        ts = struct.unpack('<I', payload[0:4])[0]
        dist = struct.unpack('<5H', payload[4:4+10])
        line = struct.unpack('<4H', payload[14:14+8])
        enemy_angle = struct.unpack('<f', payload[22:26])[0]
        v_e = struct.unpack('<f', payload[26:30])[0]
        v_0 = struct.unpack('<f', payload[30:34])[0]
        pitch = struct.unpack('<f', payload[34:38])[0]
        roll = struct.unpack('<f', payload[38:42])[0]
        state = payload[42]
        flags = payload[43]

        # Cập nhật đồ thị
        self.telemetry_data['timestamp'].append(ts)
        self.telemetry_data['dist0'].append(dist[0])
        self.telemetry_data['enemy_angle'].append(enemy_angle)
        self.telemetry_data['v_e'].append(v_e)
        self.telemetry_data['state'].append(state)

        # Giới hạn số điểm
        for key in self.telemetry_data:
            if len(self.telemetry_data[key]) > self.max_points:
                self.telemetry_data[key] = self.telemetry_data[key][-self.max_points:]

        # Cập nhật giao diện (chạy trên thread chính)
        self.root.after(0, self.update_plots)
        # Hiển thị state text
        state_names = ["IDLE","INIT_DELAY","STRIKE","FLANK_FRONT","FLANK_SIDE","FLANK_REAR",
                       "ATK_LIFT","FEINT","DELAY_RUSH","LOCK","BRAKE","ANTI_PUSH",
                       "SIDE_GUARD","REAR_GUARD","EDGE_AVOID","ANTI_LIFT","LAST_STAND",
                       "RECOVER","SEARCH"]
        state_str = state_names[state] if state < len(state_names) else f"UNK({state})"
        self.root.after(0, lambda: self.state_text.set(f"State: {state_str} | v_e: {v_e:.1f} mm/s"))

    def process_param_response(self, payload):
        # payload: list of (param_id:2, value:4) repeated
        for i in range(0, len(payload), 6):
            if i+5 >= len(payload):
                break
            pid = struct.unpack('<H', payload[i:i+2])[0]
            val = struct.unpack('<f', payload[i+2:i+6])[0]
            if pid in PARAM_NAMES:
                # Cập nhật slider/entry tương ứng (nếu có)
                self.root.after(0, lambda p=pid, v=val: self.update_param_widget(p, v))

    def update_param_widget(self, pid, value):
        # Tìm widget trong dictionary và cập nhật giá trị
        if hasattr(self, 'param_vars') and pid in self.param_vars:
            self.param_vars[pid].set(value)

    # Tab Biểu đồ 
    def setup_plot_tab(self):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(6, 8))
        self.ax_dist = ax1
        self.ax_angle = ax2
        self.ax_ve = ax3
        fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # Khởi tạo các line
        self.line_dist, = self.ax_dist.plot([], [], 'b-', label='Khoảng cách (mm)')
        self.line_angle, = self.ax_angle.plot([], [], 'r-', label='Góc (độ)')
        self.line_ve, = self.ax_ve.plot([], [], 'g-', label='Vận tốc tiếp cận (mm/s)')

        self.ax_dist.set_ylabel('mm')
        self.ax_angle.set_ylabel('deg')
        self.ax_ve.set_ylabel('mm/s')
        self.ax_ve.set_xlabel('Thời gian (sample)')

        for ax in (self.ax_dist, self.ax_angle, self.ax_ve):
            ax.legend(loc='upper right')
            ax.grid(True)

    def update_plots(self):
        t = list(range(len(self.telemetry_data['timestamp'])))
        if t:
            self.line_dist.set_data(t, self.telemetry_data['dist0'])
            self.line_angle.set_data(t, self.telemetry_data['enemy_angle'])
            self.line_ve.set_data(t, self.telemetry_data['v_e'])

            self.ax_dist.relim(); self.ax_dist.autoscale_view()
            self.ax_angle.relim(); self.ax_angle.autoscale_view()
            self.ax_ve.relim(); self.ax_ve.autoscale_view()
            self.canvas.draw_idle()

    # Tab Tuning 
    def setup_param_tab(self):
        self.param_vars = {}
        row = 0
        for pid, name in PARAM_NAMES.items():
            ttk.Label(self.param_frame, text=name).grid(row=row, column=0, sticky='w', padx=5, pady=2)
            var = tk.DoubleVar()
            entry = ttk.Entry(self.param_frame, textvariable=var, width=10)
            entry.grid(row=row, column=1, padx=5, pady=2)
            # Nút Set
            btn = ttk.Button(self.param_frame, text="Gửi", command=lambda p=pid, v=var: self.send_param_set(p, v.get()))
            btn.grid(row=row, column=2, padx=5)
            self.param_vars[pid] = var
            row += 1

        ttk.Button(self.param_frame, text="Yêu cầu đọc lại thông số từ robot", command=self.send_param_get).grid(row=row, column=0, columnspan=3, pady=10)

# Main

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotTelemetryUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect_serial(), root.destroy()))
    root.mainloop()