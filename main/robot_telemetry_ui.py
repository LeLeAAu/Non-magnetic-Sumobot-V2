import serial
import serial.tools.list_ports
import threading
import struct
import time
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Định nghĩa cấu trúc packet 
TELEMETRY_TYPE = 0x01
PARAM_SET_TYPE = 0x02
PARAM_GET_TYPE = 0x03

# Mapping tham số (cùng ID với firmware)
PARAM_IDS = {
    'WARN_DIST': 1, 'STRIKE_DIST': 2, 'PWM_MAX': 3, 'PWM_STRIKE_HOLD': 4,
    'PWM_HIGH': 5, 'PWM_MED': 6, 'PWM_LOW': 7, 'V_MAX_60': 8,
    'OMEGA_60': 9, 'KP_STEERING': 10, 'FEINT_CHANCE': 11, 'ATK_LOCK_TIME': 12,
    'TCRT_EDGE_TH': 13, 'TOF_OFFSET_0': 14, 'TOF_OFFSET_1': 15,
    'TOF_OFFSET_2': 16, 'TOF_OFFSET_3': 17, 'TOF_OFFSET_4': 18,
}

# Tên tham số (hiển thị)
PARAM_NAMES = {
    1: 'WARN_DIST (mm)', 2: 'STRIKE_DIST (mm)', 3: 'PWM_MAX', 4: 'PWM_STRIKE_HOLD',
    5: 'PWM_HIGH', 6: 'PWM_MED', 7: 'PWM_LOW', 8: 'V_MAX_60 (mm/s)',
    9: 'OMEGA_60 (deg/s)', 10: 'KP_STEERING', 11: 'FEINT_CHANCE (%)', 12: 'ATK_LOCK_TIME (ms)',
    13: 'TCRT_EDGE_TH (0-4095)', 14: 'OFFSET d0 (mm)', 15: 'OFFSET d1 (mm)',
    16: 'OFFSET d2 (mm)', 17: 'OFFSET d3 (mm)', 18: 'OFFSET d4 (mm)',
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
        self.root.title("Sumobot Telemetry & Tuning")
        self.root.geometry("1200x700") # Kích thước cửa sổ mặc định
        self.serial_port = None
        self.running = False
        
        # Dữ liệu cho Plot
        self.telemetry_data = {'timestamp': [], 'enemy_angle': []}
        self.max_points = 100

        # Tạo notebook (tab)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True)

        # Tab 1: Đồ thị & Slider (Thiết kế mới)
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

        self.recording = False
        self.record_data = []
        
        self.sim_inject_active = True
        self.root.after(100, self.send_sim_inject_loop)

    # TAB 1: GIAO DIỆN MỚI
    def send_sim_inject_loop(self):
        # Kiểm tra xem cửa sổ chính còn tồn tại không, nếu đã bị đóng thì ngắt loop luôn
        if not self.root.winfo_exists():
            return
        # Bắn dữ liệu ép thông số xuống xe mỗi 100ms
        if self.running and self.serial_port and self.serial_port.is_open:
            try:
                payload = struct.pack('<11H', 
                    self.slider_vars["d0"].get(), self.slider_vars["d1"].get(),
                    self.slider_vars["d2"].get(), self.slider_vars["d3"].get(),
                    self.slider_vars["d4"].get(),
                    self.slider_vars["fl"].get(), self.slider_vars["fr"].get(),
                    self.slider_vars["bl"].get(), self.slider_vars["br"].get(),
                    self.slider_vars["detect"].get(),
                    self.ttp223_state
                )
                frame = bytearray([0x05, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
                crc = crc16_ccitt(frame)
                frame += struct.pack('<H', crc)
                cobs_frame = self.cobs_encode(frame)
                self.serial_port.write(b'\x00' + cobs_frame + b'\x00')
            except Exception as e:
                pass
        self.root.after(100, self.send_sim_inject_loop)
        
    def setup_plot_tab(self):
        # Frame trên: Chứa các Sliders
        top_frame = ttk.Frame(self.plot_frame)
        top_frame.pack(side=tk.TOP, fill=tk.X, pady=20)
        
        # Hàm tạo slider dọc
        def create_v_slider(parent, label, max_val):
            frame = ttk.Frame(parent)
            # Dùng tk.Scale thay vì ttk.Scale để dễ dàng hiển thị số và kéo
            var = tk.IntVar()
            slider = tk.Scale(frame, from_=max_val, to=0, orient="vertical", 
                              variable=var, length=250, tickinterval=max_val, 
                              showvalue=True, width=20, sliderrelief='flat')
            slider.pack()
            ttk.Label(frame, text=label).pack(pady=5)
            return frame, var

        # Khởi tạo các biến lưu trữ giá trị slider
        self.slider_vars = {}
        
        # Sắp xếp theo đúng thứ tự trong hình ảnh của bạn
        sliders_config = [
            ("Khoảng cách d3", 2000, "d3"),
            ("Khoảng cách d1", 2000, "d1"),
            ("Khoảng cách d0", 2000, "d0"),
            ("Khoảng cách d2", 2000, "d2"),
            ("Khoảng cách d4", 2000, "d4"),
            ("TCRT_BL", 4096, "bl"),
            ("TCRT_BR", 4096, "br"),
            ("TCRT_FL", 4096, "fl"),
            ("TCRT_FR", 4096, "fr"),
        ]

        for i, (label, max_val, key) in enumerate(sliders_config):
            frm, var = create_v_slider(top_frame, label, max_val)
            frm.grid(row=0, column=i, padx=15)
            self.slider_vars[key] = var

        # Căn giữa các slider trong top_frame
        for i in range(len(sliders_config)):
            top_frame.columnconfigure(i, weight=1)

        # Frame dưới: Chứa Đồ thị, Label STATE và Slider TCRT_DETECT
        bottom_frame = ttk.Frame(self.plot_frame)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=20, pady=20)

        # 1. Đồ thị Góc (Bên trái)
        plot_container = ttk.Frame(bottom_frame)
        plot_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        fig, self.ax_angle = plt.subplots(figsize=(5, 3))
        fig.tight_layout(pad=2.0)
        self.canvas = FigureCanvasTkAgg(fig, master=plot_container)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.line_angle, = self.ax_angle.plot([], [], 'r-', label='Góc (độ)')
        self.ax_angle.set_ylabel('deg')
        self.ax_angle.legend(loc='upper right')
        self.ax_angle.grid(True)

        # 2. Text STATE (Ở giữa)
        state_container = ttk.Frame(bottom_frame)
        state_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.state_large_label = ttk.Label(state_container, text="STATE:\n--", font=('Arial', 24, 'bold'), justify='left')
        self.state_large_label.pack(expand=True)

        # 3. Slider TCRT_DETECT (Bên phải)
        detect_container, self.slider_vars["detect"] = create_v_slider(bottom_frame, "TCRT_DETECT", 4096)
        detect_container.pack(side=tk.RIGHT, padx=50)
        
        # 4. TTP223 Button 
        ttp_frame = ttk.Frame(bottom_frame)
        ttp_frame.pack(side=tk.RIGHT, padx=30, pady=50) 

        ttk.Label(ttp_frame, text="TTP223", font=('Arial', 18)).pack(side=tk.LEFT, padx=10)

        self.ttp223_state = 0
        self.ttp_btn = tk.Button(ttp_frame, bg="red", width=3, height=1, relief="raised", command=self.toggle_ttp223)
        self.ttp_btn.pack(side=tk.LEFT)

        self.ttp_lbl_status = ttk.Label(ttp_frame, text="OFF", font=('Arial', 18))
        self.ttp_lbl_status.pack(side=tk.LEFT, padx=10)

    def toggle_ttp223(self):
        self.ttp223_state = 1
        self.ttp_btn.config(bg="green", relief="sunken")
        self.ttp_lbl_status.config(text="ON")
        # Tự động nảy về OFF sau 500ms
        self.root.after(500, self.reset_ttp223)

    def reset_ttp223(self):
        self.ttp223_state = 0
        self.ttp_btn.config(bg="red", relief="raised")
        self.ttp_lbl_status.config(text="OFF")
        
    def update_ui_elements(self, dist, line, enemy_angle, v_e, state_str, tcrt_detect_val):
        # Hàm này sẽ được gọi bằng self.root.after để chạy trên main thread

        # Cập nhật Label
        self.state_large_label.config(text=f"STATE:\n{state_str}\n\nv_e: {v_e:.1f}")
        self.state_text.set(f"State: {state_str} | v_e: {v_e:.1f} mm/s")

        # Cập nhật Plot
        t = list(range(len(self.telemetry_data['timestamp'])))
        if t:
            self.line_angle.set_data(t, self.telemetry_data['enemy_angle'])
            self.ax_angle.relim()
            self.ax_angle.autoscale_view()
            self.canvas.draw_idle()


    # CÁC TAB CÒN LẠI & LOGIC
    def setup_serial_tab(self):
        ttk.Label(self.serial_frame, text="Cổng COM:").grid(row=0, column=0, padx=5, pady=5)
        self.combo_port = ttk.Combobox(self.serial_frame, values=self.get_serial_ports())
        self.combo_port.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(self.serial_frame, text="Kết nối", command=self.connect_serial).grid(row=0, column=2, padx=5)
        ttk.Button(self.serial_frame, text="Ngắt kết nối", command=self.disconnect_serial).grid(row=0, column=3, padx=5)
        self.status_label = ttk.Label(self.serial_frame, text="Chưa kết nối", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=4, pady=10)
        ttk.Button(self.serial_frame, text="Record", command=self.start_record).grid(row=3, column=0)
        ttk.Button(self.serial_frame, text="Stop", command=self.stop_record).grid(row=3, column=1)

        # Trạng thái ở tab Serial
        self.state_text = tk.StringVar(value="State: --")
        ttk.Label(self.serial_frame, textvariable=self.state_text, font=('Arial', 12)).grid(row=2, column=0, columnspan=4)

    def start_record(self):
        self.recording = True
        self.record_data = []

    def stop_record(self):
        self.recording = False
        if self.record_data:
            import csv
            with open("telemetry_log.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "dist0", "enemy_angle", "v_e", "state"])
                writer.writerows(self.record_data)
            print("Saved to telemetry_log.csv")

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
            self.send_param_get()
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
        if not self.serial_port or not self.serial_port.is_open: return
        frame = bytearray([PARAM_GET_TYPE, 0, 0])
        crc = crc16_ccitt(frame)
        frame += struct.pack('<H', crc)
        cobs_frame = self.cobs_encode(frame)
        self.serial_port.write(b'\x00' + cobs_frame + b'\x00')

    def send_param_set(self, param_id, value):
        if not self.serial_port or not self.serial_port.is_open: return
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
        out.append(0x01) 
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
                                            elif msg_type == 0x04: 
                                                self.process_param_response(decoded[3:3+pkt_len])
                            in_packet = False
                        elif in_packet:
                            rx_buf.append(byte)
                except Exception as e:
                    pass
            time.sleep(0.001)

    def process_telemetry(self, payload):
        if len(payload) < 46: return
        ts = struct.unpack('<I', payload[0:4])[0]
        dist = struct.unpack('<5H', payload[4:4+10])
        line = struct.unpack('<4H', payload[14:14+8])
        enemy_angle = struct.unpack('<f', payload[22:26])[0]
        v_e = struct.unpack('<f', payload[26:30])[0]
        state = payload[42]
        
        # Dữ liệu TCRT Detect chưa có trong Telemetry struct cũ, 
        # Tạm lấy line[0] làm minh hoạ, nếu firmware bạn thêm tcrt_detect thì thay vào đây
        tcrt_detect_val = line[0] 

        self.telemetry_data['timestamp'].append(ts)
        self.telemetry_data['enemy_angle'].append(enemy_angle)

        if len(self.telemetry_data['timestamp']) > self.max_points:
            self.telemetry_data['timestamp'] = self.telemetry_data['timestamp'][-self.max_points:]
            self.telemetry_data['enemy_angle'] = self.telemetry_data['enemy_angle'][-self.max_points:]

        state_names = ["IDLE","INIT_DELAY","STRIKE","FLANK_FRONT","FLANK_SIDE","FLANK_REAR",
                       "ATK_LIFT","FEINT","DELAY_RUSH","LOCK","BRAKE","ANVIL_BRK",
                       "ANTI_PUSH","SIDE_GUARD","REAR_GUARD","EDGE_AVOID","ANTI_LIFT",
                       "LAST_STAND","RECOVER","SEARCH"]
        state_str = state_names[state] if state < len(state_names) else f"UNK({state})"

        # Gọi hàm update UI trên main thread
        self.root.after(0, lambda: self.update_ui_elements(dist, line, enemy_angle, v_e, state_str, tcrt_detect_val))

        if self.recording:
            self.record_data.append([ts, dist[0], enemy_angle, v_e, state])

    def process_param_response(self, payload):
        for i in range(0, len(payload), 6):
            if i+5 >= len(payload): break
            pid = struct.unpack('<H', payload[i:i+2])[0]
            val = struct.unpack('<f', payload[i+2:i+6])[0]
            if pid in PARAM_NAMES:
                self.root.after(0, lambda p=pid, v=val: self.update_param_widget(p, v))

    def update_param_widget(self, pid, value):
        if hasattr(self, 'param_vars') and pid in self.param_vars:
            self.param_vars[pid].set(value)

    def setup_param_tab(self):
        self.param_vars = {}
        row = 0
        for pid, name in PARAM_NAMES.items():
            ttk.Label(self.param_frame, text=name).grid(row=row, column=0, sticky='w', padx=5, pady=2)
            var = tk.DoubleVar()
            entry = ttk.Entry(self.param_frame, textvariable=var, width=10)
            entry.grid(row=row, column=1, padx=5, pady=2)
            btn = ttk.Button(self.param_frame, text="Gửi", command=lambda p=pid, v=var: self.send_param_set(p, v.get()))
            btn.grid(row=row, column=2, padx=5)
            self.param_vars[pid] = var
            row += 1
        ttk.Button(self.param_frame, text="Yêu cầu đọc lại", command=self.send_param_get).grid(row=row, column=0, columnspan=3, pady=10)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotTelemetryUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect_serial(), root.destroy()))
    root.mainloop()