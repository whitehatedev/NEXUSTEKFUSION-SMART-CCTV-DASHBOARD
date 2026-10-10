"""
NEXUSTEKFUSION SMART CCTV DASHBOARD
====================================
- Auto-detects and prints the PC's LAN IP
- Receives KY-037 mic audio from ESP32 DevKit   (UDP 5005)
- Receives sensor data from ESP32 DevKit        (UDP 5002)
- Receives video stream from ESP32-CAM          (HTTP :81/stream)
- Sends PC microphone audio to ESP32 DevKit     (UDP 5000)
- Professional voice modulator + noise reduction

Required packages:
    pip install PyQt5 opencv-python sounddevice numpy scipy
"""

import sys
import cv2
import socket
import threading
import time
import os
import queue
import numpy as np
import sounddevice as sd

from collections import deque
from datetime import datetime

from scipy.signal import iirfilter, lfilter, lfilter_zi

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QLineEdit, QVBoxLayout, QHBoxLayout,
    QMessageBox, QFrame, QComboBox, QSlider, QGroupBox, QCheckBox
)

# ============================================================
# CONFIGURATION  *** CHANGE THESE TWO IPs ***
# ============================================================
CAMERA_IP  = "192.168.1.8"     # ESP32-CAM
ESP32_IP   = "192.168.1.7"     # ESP32 DevKit

# ============================================================
# UDP PORTS
# ============================================================
MIC_PORT      = 5005
SPEAKER_PORT  = 5000
SENSOR_PORT   = 5002

# ============================================================
# MIC PACKET FORMAT (must match ESP32 firmware)
# ============================================================
SAMPLE_RATE        = 16000
SAMPLES_PER_PACKET = 256
HEADER_SIZE        = 4
EXPECTED_MIC_PKT   = HEADER_SIZE + SAMPLES_PER_PACKET * 2

# ============================================================
# RECORDING FOLDER
# ============================================================
RECORD_FOLDER = "CCTV_Recordings"
os.makedirs(RECORD_FOLDER, exist_ok=True)


# ============================================================
# PC IP HELPERS
# ============================================================
def get_pc_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def print_pc_ips():
    primary = get_pc_ip()
    print()
    print("=" * 62)
    print(" COMPUTER NETWORK INFORMATION")
    print("=" * 62)
    print(f" Primary IPv4 : {primary}")
    parts = primary.split(".")
    if len(parts) == 4:
        print(f' IPAddress PC_IP ({parts[0]}, {parts[1]}, {parts[2]}, {parts[3]});')
    print("=" * 62)
    print()
    return primary


# ============================================================
# VOICE PROCESSOR (professional pipeline)
# ============================================================
class VoiceProcessor:
    MODES = [
        "Normal", "Robot", "Deep (Vader)", "Chipmunk",
        "Echo", "Reverb", "Radio", "Whisper",
        "Chorus", "Underwater", "Alien", "Megaphone", "Custom Pitch",
    ]

    def __init__(self, fs=SAMPLE_RATE, block=256):
        self.fs = fs
        self.block = block

        self.dc_b, self.dc_a = iirfilter(1, 20 / (fs / 2), btype='high', ftype='butter')
        self.dc_zi = lfilter_zi(self.dc_b, self.dc_a) * 0.0

        self.notch_b, self.notch_a = iirfilter(
            2, [(48 / (fs / 2)), (52 / (fs / 2))], btype='bandstop', ftype='butter')
        self.notch_zi = lfilter_zi(self.notch_b, self.notch_a) * 0.0

        self.hp_b, self.hp_a = iirfilter(4, 120 / (fs / 2), btype='high', ftype='butter')
        self.hp_zi = lfilter_zi(self.hp_b, self.hp_a) * 0.0

        self.lp_b, self.lp_a = iirfilter(4, 7000 / (fs / 2), btype='low', ftype='butter')
        self.lp_zi = lfilter_zi(self.lp_b, self.lp_a) * 0.0

        self.fft_size = 512
        self.window   = np.hanning(self.fft_size).astype(np.float32)
        self.noise_psd = None
        self.noise_alpha = 0.97
        self.nr_enabled = True
        self.nr_strength = 0.85

        self.ola_in  = np.zeros(self.fft_size, dtype=np.float32)
        self.ola_out = np.zeros(self.fft_size, dtype=np.float32)

        self.agc_target_rms = 0.08
        self.agc_gain = 1.0
        self.agc_attack = 0.20
        self.agc_release = 0.02

        self.limiter_threshold = 0.95
        self.limiter_env = 0.0

        self.effect = "Normal"
        self.pitch_semitones = 0
        self.osc_phase = 0.0

        self.echo_buf = np.zeros(int(0.4 * fs), dtype=np.float32)
        self.echo_idx = 0

        self.rev_delays_ms = [29.7, 37.1, 41.1, 43.7]
        self.rev_bufs = [np.zeros(int(fs * d / 1000), dtype=np.float32) for d in self.rev_delays_ms]
        self.rev_idxs = [0]*4
        self.rev_fb = [0.75, 0.78, 0.72, 0.70]
        self.rev_ap1 = np.zeros(int(fs * 0.005), dtype=np.float32)
        self.rev_ap2 = np.zeros(int(fs * 0.0017), dtype=np.float32)
        self.rev_i1 = 0
        self.rev_i2 = 0

        self.chorus_buf = np.zeros(int(0.05 * fs), dtype=np.float32)
        self.chorus_i = 0
        self.trem_phase = 0.0

        self.ps_buf_a = deque(maxlen=block * 4)
        self.prev_tail = np.zeros(block // 2, dtype=np.float32)

    def _apply_filters(self, x):
        x, self.dc_zi = lfilter(self.dc_b, self.dc_a, x, zi=self.dc_zi)
        x, self.notch_zi = lfilter(self.notch_b, self.notch_a, x, zi=self.notch_zi)
        x, self.hp_zi = lfilter(self.hp_b, self.hp_a, x, zi=self.hp_zi)
        x, self.lp_zi = lfilter(self.lp_b, self.lp_a, x, zi=self.lp_zi)
        return x.astype(np.float32)

    def _noise_reduce(self, x):
        if not self.nr_enabled:
            return x
        n = len(x)
        self.ola_in[:-n] = self.ola_in[n:]
        self.ola_in[-n:] = x
        frame = self.ola_in * self.window
        spec  = np.fft.rfft(frame)
        mag   = np.abs(spec)
        phase = np.angle(spec)

        if self.noise_psd is None:
            self.noise_psd = mag.copy()
        else:
            self.noise_psd = np.minimum(
                self.noise_psd * 1.0005,
                mag * self.noise_alpha + self.noise_psd * (1 - self.noise_alpha))

        noise_mag = self.noise_psd * self.nr_strength
        clean_mag = np.maximum(mag - noise_mag, 0.0)
        floor = mag * 0.05
        clean_mag = np.maximum(clean_mag, floor)

        clean_frame = np.fft.irfft(clean_mag * np.exp(1j * phase)).astype(np.float32)
        clean_frame *= self.window
        self.ola_out[:-n] = self.ola_out[n:]
        self.ola_out[-n:] = 0.0
        self.ola_out += clean_frame[-n:]
        return self.ola_out[-n:].copy() / (self.window[-n:] + 1e-6)

    def _agc(self, x):
        rms = float(np.sqrt(np.mean(x * x) + 1e-12))
        if rms > 1e-4:
            desired = self.agc_target_rms / rms
            if desired > 1.0:
                self.agc_gain += (desired - self.agc_gain) * self.agc_attack
            else:
                self.agc_gain += (desired - self.agc_gain) * self.agc_release
            self.agc_gain = np.clip(self.agc_gain, 0.1, 20.0)
        return x * self.agc_gain

    def _limiter(self, x):
        env = np.abs(x)
        for i in range(len(env)):
            if env[i] > self.limiter_env:
                self.limiter_env += 0.5 * (env[i] - self.limiter_env)
            else:
                self.limiter_env += 0.01 * (env[i] - self.limiter_env)
        gain = np.ones_like(x)
        if self.limiter_env > self.limiter_threshold:
            gain = self.limiter_threshold / (self.limiter_env + 1e-9)
        y = x * gain
        return np.tanh(y * 1.2).astype(np.float32) / np.tanh(1.2)

    def _fx_robot(self, x):
        n = len(x)
        t = (np.arange(n) + self.osc_phase) / self.fs
        self.osc_phase += n
        carrier = np.cos(2 * np.pi * 60.0 * t).astype(np.float32)
        return np.tanh(x * carrier * 1.5) * 0.85

    def _fx_pitch_shift(self, x, semitones):
        if semitones == 0:
            return x
        ratio = 2.0 ** (semitones / 12.0)
        n = len(x)
        for s in x:
            self.ps_buf_a.append(s)
        if len(self.ps_buf_a) < n * 2:
            return x
        buf = np.array(self.ps_buf_a, dtype=np.float32)
        read_len = min(int(n * ratio), len(buf) - 1)
        idx = np.linspace(0, read_len - 1, n)
        idx_f = np.floor(idx).astype(int)
        idx_c = np.minimum(idx_f + 1, read_len - 1)
        frac = idx - idx_f
        y = buf[idx_f] * (1 - frac) + buf[idx_c] * frac
        cf = self.block // 2
        fade = np.linspace(0, 1, cf, dtype=np.float32)
        y[:cf] = y[:cf] * fade + self.prev_tail * (1 - fade)
        self.prev_tail = y[-cf:].copy()
        return y.astype(np.float32)

    def _fx_echo(self, x, delay_ms=260, feedback=0.45, mix=0.5):
        n = len(x)
        d = int(self.fs * delay_ms / 1000.0) % len(self.echo_buf)
        y = np.empty(n, dtype=np.float32)
        for i in range(n):
            delayed = self.echo_buf[(self.echo_idx - d) % len(self.echo_buf)]
            y[i] = x[i] + feedback * delayed
            self.echo_buf[self.echo_idx] = y[i]
            self.echo_idx = (self.echo_idx + 1) % len(self.echo_buf)
        return (x * (1 - mix) + y * mix).astype(np.float32)

    def _fx_reverb(self, x, mix=0.35):
        n = len(x)
        y = np.zeros(n, dtype=np.float32)
        for c in range(4):
            buf = self.rev_bufs[c]
            idx = self.rev_idxs[c]
            fb  = self.rev_fb[c]
            for i in range(n):
                delayed = buf[idx]
                buf[idx] = x[i] + fb * delayed
                idx = (idx + 1) % len(buf)
                y[i] += delayed * 0.25
            self.rev_idxs[c] = idx
        g = 0.7
        for i in range(n):
            bufout = self.rev_ap1[self.rev_i1]
            self.rev_ap1[self.rev_i1] = y[i] + g * bufout
            y[i] = -g * y[i] + bufout
            self.rev_i1 = (self.rev_i1 + 1) % len(self.rev_ap1)
        g = 0.6
        for i in range(n):
            bufout = self.rev_ap2[self.rev_i2]
            self.rev_ap2[self.rev_i2] = y[i] + g * bufout
            y[i] = -g * y[i] + bufout
            self.rev_i2 = (self.rev_i2 + 1) % len(self.rev_ap2)
        return (x * (1 - mix) + y * mix).astype(np.float32)

    def _fx_radio(self, x):
        b_hp, a_hp = iirfilter(2, 300 / (self.fs / 2), btype='high', ftype='butter')
        b_lp, a_lp = iirfilter(2, 3400 / (self.fs / 2), btype='low', ftype='butter')
        y = lfilter(b_hp, a_hp, x)
        y = lfilter(b_lp, a_lp, y)
        return np.tanh(y * 3.0).astype(np.float32) * 0.7

    def _fx_whisper(self, x):
        b_hp, a_hp = iirfilter(2, 1200 / (self.fs / 2), btype='high', ftype='butter')
        y = lfilter(b_hp, a_hp, x)
        env = np.abs(y) + 1e-6
        gain = np.clip(0.06 / env, 0.0, 4.0).astype(np.float32)
        return (y * gain).astype(np.float32)

    def _fx_chorus(self, x, depth_ms=15, rate=0.7):
        n = len(x)
        y = np.empty(n, dtype=np.float32)
        for i in range(n):
            t = (self.chorus_i + i) / self.fs
            delay = int(self.fs * (depth_ms * 0.5 * (1 + np.sin(2*np.pi*rate*t))) / 1000.0)
            read = (self.chorus_i + i - delay) % len(self.chorus_buf)
            y[i] = x[i] * 0.7 + self.chorus_buf[read] * 0.6
            self.chorus_buf[(self.chorus_i + i) % len(self.chorus_buf)] = x[i]
        self.chorus_i = (self.chorus_i + n) % len(self.chorus_buf)
        return y

    def _fx_underwater(self, x):
        b, a = iirfilter(2, 600 / (self.fs / 2), btype='low', ftype='butter')
        y = lfilter(b, a, x)
        t = (np.arange(len(x)) + self.trem_phase) / self.fs
        self.trem_phase += len(x)
        trem = 0.55 + 0.45 * np.sin(2 * np.pi * 3.5 * t)
        return (y * trem).astype(np.float32)

    def _fx_alien(self, x):
        n = len(x)
        t = (np.arange(n) + self.osc_phase) / self.fs
        self.osc_phase += n
        carrier = np.cos(2 * np.pi * 25.0 * t).astype(np.float32)
        mod     = np.sin(2 * np.pi * 25.0 * t).astype(np.float32)
        y = x * carrier + x * mod * 0.3
        trem = 0.7 + 0.3 * np.sin(2 * np.pi * 6.0 * t)
        return (y * trem).astype(np.float32)

    def _fx_megaphone(self, x):
        b_hp, a_hp = iirfilter(2, 500 / (self.fs / 2), btype='high', ftype='butter')
        b_lp, a_lp = iirfilter(2, 3000 / (self.fs / 2), btype='low', ftype='butter')
        y = lfilter(b_hp, a_hp, x)
        y = lfilter(b_lp, a_lp, y)
        return np.clip(y * 4.0, -0.9, 0.9).astype(np.float32)

    def process(self, x, effect, pitch_semitones=0):
        x = x.astype(np.float32).copy()
        x = self._apply_filters(x)
        x = self._noise_reduce(x)
        try:
            if   effect == "Robot":        x = self._fx_robot(x)
            elif effect == "Deep (Vader)": x = self._fx_pitch_shift(x, -6)
            elif effect == "Chipmunk":     x = self._fx_pitch_shift(x, +7)
            elif effect == "Echo":         x = self._fx_echo(x)
            elif effect == "Reverb":       x = self._fx_reverb(x)
            elif effect == "Radio":        x = self._fx_radio(x)
            elif effect == "Whisper":      x = self._fx_whisper(x)
            elif effect == "Chorus":       x = self._fx_chorus(x)
            elif effect == "Underwater":   x = self._fx_underwater(x)
            elif effect == "Alien":        x = self._fx_alien(x)
            elif effect == "Megaphone":    x = self._fx_megaphone(x)
            elif effect == "Custom Pitch": x = self._fx_pitch_shift(x, pitch_semitones)
        except Exception as e:
            print("FX error:", e)
        x = self._agc(x)
        x = self._limiter(x)
        x = np.nan_to_num(x, nan=0.0, posinf=1.0, neginf=-1.0)
        np.clip(x, -1.0, 1.0, out=x)
        return x.astype(np.float32)


# ============================================================
# MAIN WINDOW
# ============================================================
class CCTVWindow(QMainWindow):

    sensor_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        self.pc_ip = get_pc_ip()

        self.cap = None
        self.camera_connected = False

        self.recording = False
        self.video_writer = None
        self.recording_path = ""

        self.mic_running = False
        self.talk_running = False

        self.mic_socket     = None
        self.speaker_socket = None
        self.sensor_socket  = None

        self.audio_queue  = queue.Queue(maxsize=40)
        self.audio_stream = None

        self.processor = VoiceProcessor(SAMPLE_RATE, SAMPLES_PER_PACKET)
        self.mod_mode = "Normal"
        self.pitch_semitones = 0
        self.nr_enabled = True

        # Sensor state
        self.temperature  = 0.0
        self.humidity     = 0.0
        self.thermocouple = 0.0
        self.mq2 = 0
        self.mq7 = 0
        self.vibration = 0
        self.alarm = 0
        self.rssi = 0
        self.mic_rms = 0.0
        self.mic_dc  = 0.0

        self.last_seq = -1
        self.dropped  = 0

        self.setWindowTitle("NEXUSTEKFUSION SMART CCTV")
        self.setGeometry(60, 30, 1600, 980)
        self.setMinimumSize(1200, 780)

        self.create_style()
        self.create_ui()
        self.setup_udp()
        self.start_audio_stream()

        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_video)
        self.video_timer.start(30)

        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self.update_clock)
        self.clock_timer.start(1000)

        self.sensor_signal.connect(self.update_sensor_ui)

        print("=" * 62)
        print(" NEXUSTEKFUSION DASHBOARD STARTED")
        print("=" * 62)
        print(f" PC IP         : {self.pc_ip}")
        print(f" Mic listen    : UDP {MIC_PORT}  (SR={SAMPLE_RATE} Hz)")
        print(f" Sensor listen : UDP {SENSOR_PORT}")
        print(f" Speaker send  : UDP {SPEAKER_PORT} -> {ESP32_IP}")
        print(f" Camera stream : http://{CAMERA_IP}:81/stream")
        print("=" * 62)

    def create_style(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #101214; }
            QWidget { background-color: #101214; color: white; font-family: Arial; }
            QLabel { color: white; }
            QPushButton {
                background-color: #24282c; color: white;
                border: 1px solid #444; border-radius: 6px;
                padding: 10px; font-size: 14px;
            }
            QPushButton:hover { background-color: #343a40; }
            QLineEdit {
                background-color: #1d2125; color: white;
                border: 1px solid #555; border-radius: 5px; padding: 8px;
            }
            QComboBox {
                background-color: #1d2125; color: white;
                border: 1px solid #555; border-radius: 5px; padding: 6px;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #30343a; border-radius: 8px;
                margin-top: 10px; padding-top: 10px;
                color: #9ac7ff; font-weight: bold;
            }
            QSlider::groove:horizontal {
                background: #30343a; height: 6px; border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #00b0ff; width: 16px; margin: -6px 0; border-radius: 8px;
            }
            QCheckBox { color: white; font-size: 13px; }
            QFrame {
                background-color: #181b1f;
                border: 1px solid #30343a; border-radius: 8px;
            }
        """)

    def create_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main = QVBoxLayout()
        central.setLayout(main)

        # Header
        header = QHBoxLayout()
        title = QLabel("NEXUSTEKFUSION")
        title.setFont(QFont("Arial", 26, QFont.Bold))
        subtitle = QLabel("SMART CCTV • SAFETY MONITORING")
        subtitle.setFont(QFont("Arial", 12))
        self.clock = QLabel()
        self.clock.setFont(QFont("Arial", 12))
        header.addWidget(title)
        header.addWidget(subtitle)
        header.addStretch()
        header.addWidget(self.clock)
        main.addLayout(header)

        # PC IP banner
        ip_banner = QFrame()
        ip_banner.setStyleSheet(
            "QFrame { background-color: #0b3d2e; border: 1px solid #0f5e46;"
            " border-radius: 8px; }")
        ip_layout = QHBoxLayout()
        ip_banner.setLayout(ip_layout)
        ip_title = QLabel("💻  YOUR PC IP:")
        ip_title.setFont(QFont("Arial", 13, QFont.Bold))
        ip_layout.addWidget(ip_title)
        self.pc_ip_label = QLabel(self.pc_ip)
        self.pc_ip_label.setFont(QFont("Consolas", 15, QFont.Bold))
        self.pc_ip_label.setStyleSheet("color: #00ff88; padding-left: 8px;")
        self.pc_ip_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        ip_layout.addWidget(self.pc_ip_label)
        ip_layout.addStretch()
        main.addWidget(ip_banner)

        # Content
        content = QHBoxLayout()

        # Video
        video_frame = QFrame()
        video_layout = QVBoxLayout()
        video_frame.setLayout(video_layout)
        self.video_label = QLabel("CAMERA OFFLINE")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(850, 480)
        self.video_label.setStyleSheet(
            "QLabel { background-color: black; border: 2px solid #333;"
            " font-size: 20px; }")
        video_layout.addWidget(self.video_label)
        self.camera_status = QLabel("● CAMERA OFFLINE")
        self.camera_status.setFont(QFont("Arial", 14, QFont.Bold))
        video_layout.addWidget(self.camera_status)
        content.addWidget(video_frame, 3)

        # Sensors
        sensor_frame = QFrame()
        sensor_layout = QVBoxLayout()
        sensor_frame.setLayout(sensor_layout)
        sensor_title = QLabel("SENSOR STATUS")
        sensor_title.setFont(QFont("Arial", 18, QFont.Bold))
        sensor_layout.addWidget(sensor_title)

        self.temp_label      = QLabel("🌡 Temperature: -- °C")
        self.humidity_label  = QLabel("💧 Humidity: -- %")
        self.tc_label        = QLabel("🔥 K-Type: -- °C")
        self.mq2_label       = QLabel("🟠 MQ-2: --")
        self.mq7_label       = QLabel("🟠 MQ-7: --")
        self.vibration_label = QLabel("📳 Vibration: --")
        self.rssi_label      = QLabel("📡 Wi-Fi: -- dBm")
        self.mic_label       = QLabel("🎙 Mic level: --")
        self.dc_label        = QLabel("⚖ Mic DC offset: --")

        for lbl in [self.temp_label, self.humidity_label, self.tc_label,
                    self.mq2_label, self.mq7_label, self.vibration_label,
                    self.rssi_label, self.mic_label, self.dc_label]:
            lbl.setFont(QFont("Arial", 13))
            sensor_layout.addWidget(lbl)

        sensor_layout.addSpacing(20)
        self.alarm_label = QLabel("SYSTEM NORMAL")
        self.alarm_label.setAlignment(Qt.AlignCenter)
        self.alarm_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.alarm_label.setMinimumHeight(60)
        sensor_layout.addWidget(self.alarm_label)
        sensor_layout.addStretch()
        content.addWidget(sensor_frame, 1)
        main.addLayout(content)

        # Voice processing panel
        mod_group = QGroupBox("🎛️  VOICE PROCESSING  (ESP32 mic → PC speakers)")
        mod_layout = QVBoxLayout()
        mod_group.setLayout(mod_layout)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Effect:"))
        self.mod_combo = QComboBox()
        self.mod_combo.addItems(VoiceProcessor.MODES)
        self.mod_combo.setCurrentText("Normal")
        self.mod_combo.currentTextChanged.connect(self.on_mod_mode_changed)
        self.mod_combo.setMinimumWidth(200)
        row1.addWidget(self.mod_combo)

        self.nr_checkbox = QCheckBox("Noise Reduction")
        self.nr_checkbox.setChecked(True)
        self.nr_checkbox.stateChanged.connect(self.on_nr_toggled)
        row1.addWidget(self.nr_checkbox)

        row1.addSpacing(20)
        row1.addWidget(QLabel("Pitch:"))
        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_slider.setMinimum(-12)
        self.pitch_slider.setMaximum(12)
        self.pitch_slider.setValue(0)
        self.pitch_slider.setTickInterval(2)
        self.pitch_slider.setTickPosition(QSlider.TicksBelow)
        self.pitch_slider.valueChanged.connect(self.on_pitch_changed)
        self.pitch_slider.setMinimumWidth(260)
        row1.addWidget(self.pitch_slider)

        self.pitch_value_label = QLabel("0 st")
        self.pitch_value_label.setFont(QFont("Consolas", 12, QFont.Bold))
        self.pitch_value_label.setStyleSheet("color: #00b0ff;")
        self.pitch_value_label.setFixedWidth(60)
        row1.addWidget(self.pitch_value_label)
        row1.addStretch()
        mod_layout.addLayout(row1)

        row2 = QHBoxLayout()
        presets = [
            ("Normal", "Normal"), ("🤖 Robot", "Robot"), ("👹 Vader", "Deep (Vader)"),
            ("🐿️ Chipmunk", "Chipmunk"), ("🔄 Echo", "Echo"), ("🏛️ Reverb", "Reverb"),
            ("📻 Radio", "Radio"), ("🤫 Whisper", "Whisper"),
            ("🌊 Underwater", "Underwater"), ("👽 Alien", "Alien"),
            ("📢 Megaphone", "Megaphone"),
        ]
        for label, mode in presets:
            btn = QPushButton(label)
            btn.setStyleSheet("padding: 6px; font-size: 12px;")
            btn.clicked.connect(lambda checked, m=mode: self.mod_combo.setCurrentText(m))
            row2.addWidget(btn)
        row2.addStretch()
        mod_layout.addLayout(row2)
        main.addWidget(mod_group)

        # Network row
        network = QHBoxLayout()
        network.addWidget(QLabel("ESP32-CAM IP:"))
        self.camera_ip_input = QLineEdit(CAMERA_IP)
        network.addWidget(self.camera_ip_input)
        network.addWidget(QLabel("ESP32 DevKit IP:"))
        self.audio_ip_input = QLineEdit(ESP32_IP)
        network.addWidget(self.audio_ip_input)
        connect_btn = QPushButton("CONNECT CAMERA")
        connect_btn.clicked.connect(self.connect_camera)
        network.addWidget(connect_btn)
        network.addWidget(QLabel("This PC IP:"))
        self.pc_ip_field = QLineEdit(self.pc_ip)
        self.pc_ip_field.setReadOnly(True)
        self.pc_ip_field.setStyleSheet(
            "background-color: #0b3d2e; color: #00ff88; font-weight: bold;")
        network.addWidget(self.pc_ip_field)
        copy_btn = QPushButton("📋 COPY")
        copy_btn.setFixedWidth(90)
        copy_btn.clicked.connect(self.copy_pc_ip)
        network.addWidget(copy_btn)
        main.addLayout(network)

        # Controls row
        controls = QHBoxLayout()
        snapshot = QPushButton("📸 SNAPSHOT")
        snapshot.clicked.connect(self.take_snapshot)
        self.record_btn = QPushButton("🔴 START RECORDING")
        self.record_btn.clicked.connect(self.toggle_recording)
        self.mic_btn = QPushButton("🎤 START CAMERA MIC")
        self.mic_btn.clicked.connect(self.toggle_camera_microphone)
        self.talk_btn = QPushButton("🔊 PUSH TO TALK")
        self.talk_btn.pressed.connect(self.start_talk)
        self.talk_btn.released.connect(self.stop_talk)
        fullscreen = QPushButton("⛶ FULLSCREEN")
        fullscreen.clicked.connect(self.toggle_fullscreen)
        disconnect = QPushButton("DISCONNECT")
        disconnect.clicked.connect(self.disconnect_camera)
        controls.addWidget(snapshot)
        controls.addWidget(self.record_btn)
        controls.addWidget(self.mic_btn)
        controls.addWidget(self.talk_btn)
        controls.addWidget(fullscreen)
        controls.addWidget(disconnect)
        main.addLayout(controls)

    # --------------------------------------------------------
    def on_mod_mode_changed(self, mode):
        self.mod_mode = mode
        print(f"Voice effect -> {mode}")

    def on_pitch_changed(self, value):
        self.pitch_semitones = value
        self.pitch_value_label.setText(f"{value:+d} st")
        if self.mod_mode != "Custom Pitch":
            self.mod_combo.setCurrentText("Custom Pitch")

    def on_nr_toggled(self, state):
        self.nr_enabled = bool(state)
        self.processor.nr_enabled = self.nr_enabled
        print(f"Noise reduction -> {'ON' if self.nr_enabled else 'OFF'}")

    def copy_pc_ip(self):
        QApplication.clipboard().setText(self.pc_ip)
        QMessageBox.information(self, "Copied", f"PC IP copied: {self.pc_ip}")

    def setup_udp(self):
        self.mic_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mic_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mic_socket.bind(("0.0.0.0", MIC_PORT))
        self.mic_socket.settimeout(0.2)

        self.speaker_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sensor_socket.bind(("0.0.0.0", SENSOR_PORT))
        self.sensor_socket.settimeout(0.2)

        threading.Thread(target=self.sensor_listener, daemon=True).start()

    def start_audio_stream(self):
        self.audio_stream = sd.OutputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype="float32",
            blocksize=SAMPLES_PER_PACKET, callback=self.audio_callback)
        self.audio_stream.start()

    def audio_callback(self, outdata, frames, time_info, status):
        if status:
            print("Audio status:", status)
        outdata.fill(0)
        try:
            audio = self.audio_queue.get_nowait()
        except queue.Empty:
            return
        count = min(frames, len(audio))
        outdata[:count, 0] = audio[:count]

    # Camera
    def connect_camera(self):
        self.disconnect_camera()
        ip = self.camera_ip_input.text().strip()
        if not ip:
            QMessageBox.warning(self, "Error", "Enter ESP32-CAM IP.")
            return
        url = f"http://{ip}:81/stream"
        print("Connecting to camera:", url)
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.cap.isOpened():
            self.camera_status.setText("● CAMERA ONLINE")
            self.camera_status.setStyleSheet("color: #00ff66;")
        else:
            self.camera_status.setText("● CAMERA CONNECTION FAILED")
            self.camera_status.setStyleSheet("color: red;")

    def update_video(self):
        if self.cap is None or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.camera_status.setText("● VIDEO LOST")
            self.camera_status.setStyleSheet("color: red;")
            return
        self.camera_status.setText("● LIVE")
        self.camera_status.setStyleSheet("color: #00ff66;")

        now = datetime.now()
        cv2.putText(frame, "NEXUSTEKFUSION CCTV", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, now.strftime("%d-%m-%Y %H:%M:%S"), (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.circle(frame, (frame.shape[1] - 35, 30), 10, (0, 0, 255), -1)
        cv2.putText(frame, "LIVE", (frame.shape[1] - 90, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"MIC FX: {self.mod_mode}  NR: {'ON' if self.nr_enabled else 'OFF'}",
                    (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2)

        if self.recording:
            if self.video_writer is None:
                height, width = frame.shape[:2]
                filename = "CCTV_" + now.strftime("%Y%m%d_%H%M%S") + ".avi"
                self.recording_path = os.path.join(RECORD_FOLDER, filename)
                codec = cv2.VideoWriter_fourcc(*"XVID")
                self.video_writer = cv2.VideoWriter(
                    self.recording_path, codec, 20.0, (width, height))
            self.video_writer.write(frame)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.video_label.setPixmap(pix)

    def update_clock(self):
        self.clock.setText(datetime.now().strftime("%d-%m-%Y  %H:%M:%S"))

    def take_snapshot(self):
        if self.cap is None:
            QMessageBox.warning(self, "Camera", "Connect camera first.")
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        path = os.path.join(RECORD_FOLDER,
                            "Snapshot_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg")
        cv2.imwrite(path, frame)
        QMessageBox.information(self, "Snapshot Saved", path)

    def toggle_recording(self):
        if not self.recording:
            self.recording = True
            self.video_writer = None
            self.record_btn.setText("⏹ STOP RECORDING")
            self.record_btn.setStyleSheet("background-color: #8b0000;")
        else:
            self.recording = False
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            self.record_btn.setText("🔴 START RECORDING")
            self.record_btn.setStyleSheet("")

    # ESP32 mic -> PC speakers
    def toggle_camera_microphone(self):
        if not self.mic_running:
            self.mic_running = True
            self.mic_btn.setText("🎤 STOP CAMERA MIC")
            threading.Thread(target=self.receive_camera_audio, daemon=True).start()
        else:
            self.mic_running = False
            self.mic_btn.setText("🎤 START CAMERA MIC")

    def receive_camera_audio(self):
        print(f"Camera mic listener started (UDP {MIC_PORT})")
        while self.mic_running:
            try:
                data, addr = self.mic_socket.recvfrom(2048)
            except socket.timeout:
                continue
            except Exception as e:
                print("MIC ERROR:", e)
                break

            if len(data) != EXPECTED_MIC_PKT:
                continue

            seq = int.from_bytes(data[:4], "little")
            if self.last_seq >= 0 and seq != (self.last_seq + 1) % (2**32):
                self.dropped += 1
            self.last_seq = seq

            pcm = np.frombuffer(data[4:], dtype="<i2").astype(np.float32) / 32768.0

            try:
                pcm = self.processor.process(pcm, self.mod_mode, self.pitch_semitones)
            except Exception as e:
                print("Processor error:", e)

            try:
                self.audio_queue.put_nowait(pcm)
            except queue.Full:
                try:
                    self.audio_queue.get_nowait()
                    self.audio_queue.put_nowait(pcm)
                except queue.Empty:
                    pass

        print(f"Camera mic listener stopped. Dropped: {self.dropped}")

    # PC mic -> ESP32 speaker
    def start_talk(self):
        if self.talk_running:
            return
        self.talk_running = True
        self.talk_btn.setText("🔊 TALKING...")
        self.talk_btn.setStyleSheet("background-color: #8b0000;")
        threading.Thread(target=self.send_pc_microphone, daemon=True).start()

    def stop_talk(self):
        self.talk_running = False
        self.talk_btn.setText("🔊 PUSH TO TALK")
        self.talk_btn.setStyleSheet("")

    def send_pc_microphone(self):
        ip = self.audio_ip_input.text().strip()
        if not ip:
            self.talk_running = False
            return

        def callback(indata, frames, time_info, status):
            if not self.talk_running:
                return
            audio = indata[:, 0].astype(np.int16).tobytes()
            try:
                self.speaker_socket.sendto(audio, (ip, SPEAKER_PORT))
            except Exception as e:
                print("Speaker send error:", e)

        try:
            with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                                dtype="int16", blocksize=SAMPLES_PER_PACKET,
                                callback=callback):
                while self.talk_running:
                    time.sleep(0.05)
        except Exception as e:
            print("PC mic error:", e)

    # Sensors
    def sensor_listener(self):
        while True:
            try:
                data, _ = self.sensor_socket.recvfrom(2048)
                text = data.decode("utf-8", errors="ignore")
                values = {}
                for part in text.split(","):
                    if "=" in part:
                        k, v = part.split("=", 1)
                        values[k] = v
                self.sensor_signal.emit(values)
            except socket.timeout:
                continue
            except Exception as e:
                print("Sensor error:", e)
                time.sleep(1)

    def update_sensor_ui(self, values):
        try:
            if "TEMP"  in values: self.temperature  = float(values["TEMP"])
            if "HUM"   in values: self.humidity     = float(values["HUM"])
            if "TC"    in values: self.thermocouple = float(values["TC"])
            if "MQ2"   in values: self.mq2          = int(values["MQ2"])
            if "MQ7"   in values: self.mq7          = int(values["MQ7"])
            if "VIB"   in values: self.vibration    = int(values["VIB"])
            if "ALARM" in values: self.alarm        = int(values["ALARM"])
            if "RSSI"  in values: self.rssi         = int(values["RSSI"])
            if "RMS"   in values: self.mic_rms      = float(values["RMS"])
            if "DC"    in values: self.mic_dc       = float(values["DC"])
        except Exception as e:
            print("Sensor parse error:", e)
            return

        self.temp_label.setText(f"🌡 Temperature: {self.temperature:.1f} °C")
        self.humidity_label.setText(f"💧 Humidity: {self.humidity:.1f} %")
        self.tc_label.setText(f"🔥 K-Type: {self.thermocouple:.1f} °C")
        self.mq2_label.setText(f"🟠 MQ-2: {self.mq2}")
        self.mq7_label.setText(f"🟠 MQ-7: {self.mq7}")

        if self.vibration:
            self.vibration_label.setText("📳 Vibration: DETECTED")
            self.vibration_label.setStyleSheet("color: orange;")
        else:
            self.vibration_label.setText("📳 Vibration: NORMAL")
            self.vibration_label.setStyleSheet("color: #00ff66;")

        self.rssi_label.setText(f"📡 Wi-Fi: {self.rssi} dBm")
        self.mic_label.setText(f"🎙 Mic level: {self.mic_rms:.1f}")
        self.dc_label.setText(f"⚖ Mic DC offset: {self.mic_dc:.0f}")

        if self.alarm:
            self.alarm_label.setText("🚨 ALARM ACTIVE")
            self.alarm_label.setStyleSheet(
                "background-color: #8b0000; color: white;"
                "border-radius: 8px; padding: 10px;")
        else:
            self.alarm_label.setText("🟢 SYSTEM NORMAL")
            self.alarm_label.setStyleSheet(
                "background-color: #064d24; color: white;"
                "border-radius: 8px; padding: 10px;")

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def disconnect_camera(self):
        if self.cap:
            self.cap.release()
            self.cap = None
        self.video_label.clear()
        self.video_label.setText("CAMERA OFFLINE")
        self.camera_status.setText("● CAMERA OFFLINE")
        self.camera_status.setStyleSheet("color: red;")

    def closeEvent(self, event):
        print("Shutting down...")
        self.mic_running = False
        self.talk_running = False
        if self.audio_stream:
            try:
                self.audio_stream.stop()
                self.audio_stream.close()
            except Exception:
                pass
        if self.cap:
            self.cap.release()
        if self.video_writer:
            self.video_writer.release()
        for sock in (self.mic_socket, self.sensor_socket, self.speaker_socket):
            try: sock.close()
            except Exception: pass
        event.accept()


# ============================================================
if __name__ == "__main__":
    print_pc_ips()
    app = QApplication(sys.argv)
    window = CCTVWindow()
    window.show()
    sys.exit(app.exec_())