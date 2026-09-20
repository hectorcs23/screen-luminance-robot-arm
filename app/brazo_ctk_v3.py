"""
================================================================================
  SIMULADOR BRAZO 2-DOF  ·  customtkinter  ·  v3
================================================================================
  - Todos los parametros via cajas de entrada de texto (sin sliders/combobox)
  - Pestaña nueva: "Datos del sensor" con graficas (barras y mapa de calor)
  - Conexion serial Arduino: el usuario teclea el puerto y baud manualmente
  - Secuencia "Esquinas + centro" con medicion VEML7700 (SCAN)
  Requisitos:
    pip install customtkinter matplotlib numpy pyserial
================================================================================
"""

import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox
import numpy as np
import threading
import time
import csv
import os
import socket
import urllib.request
import urllib.parse
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches
import matplotlib.animation as animation

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False


# ─────────────────────────────────────────────────────────────────────────── #
ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")

C_BG       = "#F4F6FA"
C_PANEL    = "#FFFFFF"
C_ACCENT   = "#2563EB"
C_ACCENT2  = "#16A34A"
C_WARN     = "#DC2626"
C_TEXT     = "#1E293B"
C_SUBTEXT  = "#64748B"
C_BORDER   = "#E2E8F0"
C_ARM1     = "#2563EB"
C_ARM2     = "#16A34A"
C_TARGET   = "#F59E0B"
C_TRAIL    = "#EF4444"
C_SCREEN   = "#DBEAFE"


# ─────────────────────────────────────────────────────────────────────────── #
#  CINEMATICA
# ─────────────────────────────────────────────────────────────────────────── #
def forward_kinematics(base, L1, L2, t1, t2):
    bx, by = base
    elbow = (bx + L1 * np.cos(t1), by + L1 * np.sin(t1))
    end = (elbow[0] + L2 * np.cos(t1 + t2),
           elbow[1] + L2 * np.sin(t1 + t2))
    return elbow, end


def inverse_kinematics(base, L1, L2, target, elbow_up=True):
    bx, by = base
    x, y = target[0] - bx, target[1] - by
    r2 = x * x + y * y
    r = np.sqrt(r2)
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None
    cos_t2 = np.clip((r2 - L1**2 - L2**2) / (2.0 * L1 * L2), -1.0, 1.0)
    t2 = np.arccos(cos_t2)
    if not elbow_up:
        t2 = -t2
    k1, k2 = L1 + L2 * np.cos(t2), L2 * np.sin(t2)
    t1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    return t1, t2


# Offsets de calibracion servo (s = angulo_articular + offset), 0..180
S1_OFFSET = 3.0      # servo hombro:  s1 = theta1 + S1_OFFSET
S2_OFFSET = 160.0     # servo codo:    s2 = theta2 + S2_OFFSET

# Angulo solido del sensor (sr) para convertir iluminancia -> luminancia
OMEGA_SR = 2*0.085   # luminancia [cd/m2] = iluminancia [lux] / OMEGA_SR


def to_servo_angles(t1_rad, t2_rad):
    t1d = np.degrees(t1_rad)
    t2d = np.degrees(t2_rad)
    s1 = int(round(np.clip(t1d + S1_OFFSET, 0, 180)))
    s2 = int(round(np.clip(t2d + S2_OFFSET, 0, 180)))
    return s1, s2


# ─────────────────────────────────────────────────────────────────────────── #
#  WIDGET AUXILIAR: LabeledEntry (caja con etiqueta + boton aplicar opcional)
# ─────────────────────────────────────────────────────────────────────────── #
class LabeledEntry(ctk.CTkFrame):
    def __init__(self, master, label, var, callback=None, width=80, **kw):
        super().__init__(master, fg_color="transparent", **kw)
        self.var = var
        self.cb = callback
        ctk.CTkLabel(self, text=label, text_color=C_TEXT,
                     font=("Helvetica", 12), anchor="w").pack(
                         side="left", fill="x", expand=True)
        self.entry = ctk.CTkEntry(self, width=width,
                                  font=("Helvetica", 12),
                                  border_color=C_BORDER)
        self.entry.insert(0, str(var.get()))
        self.entry.pack(side="right")
        self.entry.bind("<Return>", lambda e: self._apply())
        self.entry.bind("<FocusOut>", lambda e: self._apply())

    def _apply(self):
        try:
            val = float(self.entry.get())
            self.var.set(val)
            if self.cb:
                self.cb()
        except ValueError:
            self.entry.delete(0, "end")
            self.entry.insert(0, str(self.var.get()))


# ─────────────────────────────────────────────────────────────────────────── #
#  TRANSPORTE (serial USB / WiFi TCP)  -  misma interfaz que pyserial.Serial
#  El resto de la app habla con self.transport sin saber el medio fisico.
# ─────────────────────────────────────────────────────────────────────────── #
class BaseTransport:
    is_open = False

    def write(self, data):
        raise NotImplementedError

    def readline(self):
        raise NotImplementedError

    @property
    def in_waiting(self):
        return 0

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class SerialTransport(BaseTransport):
    """Envuelve un puerto serial USB (pyserial)."""

    def __init__(self, port, baud, timeout=1):
        self._ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # espera el reset del Arduino al abrir el puerto

    @property
    def is_open(self):
        return self._ser.is_open

    def write(self, data):
        return self._ser.write(data)

    def readline(self):
        return self._ser.readline()

    @property
    def in_waiting(self):
        return self._ser.in_waiting

    def reset_input_buffer(self):
        return self._ser.reset_input_buffer()

    def close(self):
        return self._ser.close()


class TcpTransport(BaseTransport):
    """Cliente TCP hacia el Arduino UNO R4 WiFi. Mantiene un buffer interno
    y reconstruye lineas terminadas en '\\n' (la fragmentacion TCP puede
    partir o juntar respuestas como SCAN_RESULT)."""

    def __init__(self, host, port, timeout=1, connect_timeout=5):
        self._buf = bytearray()
        self._open = False
        sock = socket.create_connection((host, port), timeout=connect_timeout)
        sock.settimeout(timeout)  # timeout por recv, equivalente a Serial(timeout=1)
        # Menos latencia (sin Nagle) y deteccion de caidas via keepalive
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        self._sock = sock
        self._open = True

    @property
    def is_open(self):
        return self._open

    def write(self, data):
        try:
            self._sock.sendall(data)
            return len(data)
        except OSError:
            self._open = False  # conexion caida: que la app lo detecte
            raise

    def _fill(self):
        try:
            chunk = self._sock.recv(4096)
            if chunk == b"":
                self._open = False
            else:
                self._buf.extend(chunk)
        except socket.timeout:
            pass
        except OSError:
            self._open = False

    @property
    def in_waiting(self):
        if b"\n" not in self._buf:
            self._fill()
        return len(self._buf)

    def readline(self):
        deadline = time.time() + 1.0
        while b"\n" not in self._buf and time.time() < deadline and self._open:
            self._fill()
        nl = self._buf.find(b"\n")
        if nl == -1:
            out = bytes(self._buf)
            self._buf.clear()
            return out
        out = bytes(self._buf[:nl + 1])
        del self._buf[:nl + 1]
        return out

    def reset_input_buffer(self):
        self._buf.clear()
        try:
            self._sock.setblocking(False)
            while True:
                if self._sock.recv(4096) == b"":
                    break
        except (BlockingIOError, socket.timeout, OSError):
            pass
        finally:
            try:
                self._sock.setblocking(True)
                self._sock.settimeout(1)
            except OSError:
                pass

    def close(self):
        self._open = False
        try:
            self._sock.close()
        except OSError:
            pass


# ─────────────────────────────────────────────────────────────────────────── #
#  APLICACION PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────── #
class BrazoApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Brazo Robotico 2-DOF  ·  v3")
        self.geometry("1380x860")
        self.configure(fg_color=C_BG)
        self.resizable(True, True)

        # ── Parametros del brazo ─────────────────────────────────────
        self.ancho = tk.DoubleVar(value=9)
        self.alto = tk.DoubleVar(value=6)
        self.L1 = tk.DoubleVar(value=9.7)
        self.L2 = tk.DoubleVar(value=10.2)
        self.a = tk.DoubleVar(value=12.0)
        self.elbow_up = tk.BooleanVar(value=True)

        # ── Estado ───────────────────────────────────────────────────
        self.target = None
        self.anim = None
        self.anim_running = False
        self._moving = False        # animacion de movimiento suave al clic
        self._last_s = (-1, -1)
        self._trail_x, self._trail_y = [], []
        self._sweep_angles = None
        self._seq_thread = None
        self._seq_stop = threading.Event()

        # ── Parametros de medicion VEML7700 ──────────────────────────
        self.scan_seconds = tk.IntVar(value=30)
        self.scan_samples = tk.IntVar(value=30)
        self.scan_delay_ms = tk.IntVar(value=120)   # espera entre muestras (ms)
        self.scan_results = []
        self.session_results = []     # acumulado de TODA la sesion (para CSV)
        self._csv_session_file = None  # mismo archivo CSV durante la sesion

        # ── Conexion (serial USB / WiFi TCP) ─────────────────────────
        self.transport = None
        self.conn_mode = tk.StringVar(value="Serial")   # "Serial" | "WiFi"
        self.port_var = tk.StringVar(value="COM3")
        self.baud_var = tk.StringVar(value="9600")
        self.ip_var = tk.StringVar(value="192.168.1.100")
        self.tcp_port_var = tk.StringVar(value="5000")
        self._serial_lock = threading.Lock()
        self._recent_ips = ["192.168.1.100", "192.168.4.1"]
        self._last_conn = None   # ("Serial", port, baud) | ("WiFi", host, port)

        # ── ThingSpeak ───────────────────────────────────────────────
        self.ts_enable = tk.BooleanVar(value=False)
        self.ts_key_var = tk.StringVar(value="")

        self._build_ui()
        self.redraw()
        self._compute_sweep_angles()
        self._draw_angle_chart()
        self._draw_sensor_chart()

    # ================================================================== #
    def _build_ui(self):
        # ── Columna izquierda ────────────────────────────────────────
        left = ctk.CTkScrollableFrame(self, width=320, fg_color=C_PANEL,
                                      corner_radius=12, border_width=1,
                                      border_color=C_BORDER)
        left.pack(side="left", fill="y", padx=(12, 6), pady=12)

        ctk.CTkLabel(left, text="⚙  Configuracion",
                     font=("Helvetica", 15, "bold"),
                     text_color=C_TEXT).pack(anchor="w", pady=(8, 12))

        # Geometria
        self._section(left, "Area de trabajo")
        LabeledEntry(left, "Ancho (cm)", self.ancho,
                     self._on_param_change).pack(fill="x", pady=3)
        LabeledEntry(left, "Alto (cm)", self.alto,
                     self._on_param_change).pack(fill="x", pady=3)
        LabeledEntry(left, "Separacion a (cm)", self.a,
                     self._on_param_change).pack(fill="x", pady=3)

        self._section(left, "Eslabones")
        LabeledEntry(left, "L1 - Hombro (cm)", self.L1,
                     self._on_param_change).pack(fill="x", pady=3)
        LabeledEntry(left, "L2 - Codo (cm)", self.L2,
                     self._on_param_change).pack(fill="x", pady=3)

        ctk.CTkCheckBox(left, text="Codo arriba (solucion IK)",
                        variable=self.elbow_up,
                        command=self._on_param_change,
                        checkmark_color="white",
                        fg_color=C_ACCENT).pack(anchor="w", pady=8)

        # Estado de alcance
        self._section(left, "Estado de alcance")
        self.cond_lbl = ctk.CTkLabel(left, text="",
                                     font=("Courier", 11),
                                     text_color=C_TEXT, justify="left")
        self.cond_lbl.pack(anchor="w", pady=4)

        self._section(left, "Posicion actual")
        self.angle_lbl = ctk.CTkLabel(
            left, text="Haz clic en el simulador\no usa la entrada manual.",
            font=("Courier", 11), text_color=C_SUBTEXT, justify="left")
        self.angle_lbl.pack(anchor="w", pady=4)

        # Entrada manual
        self._section(left, "Entrada manual")
        self._build_manual_panel(left)

        # Secuencias
        self._section(left, "Secuencias")
        self._build_sequences_panel(left)

        # Serial
        self._section(left, "Conexion Arduino")
        self._build_serial_panel(left)

        # ThingSpeak
        self._section(left, "ThingSpeak")
        self._build_thingspeak_panel(left)

        # ── Area derecha: notebook ───────────────────────────────────
        right = ctk.CTkFrame(self, fg_color=C_BG, corner_radius=0)
        right.pack(side="right", fill="both", expand=True,
                   padx=(0, 12), pady=12)

        self.tabview = ctk.CTkTabview(
            right, fg_color=C_PANEL,
            segmented_button_fg_color=C_BORDER,
            segmented_button_selected_color=C_ACCENT,
            segmented_button_selected_hover_color="#1D4ED8",
            segmented_button_unselected_color=C_BORDER,
            text_color=C_TEXT, corner_radius=12)
        self.tabview.pack(fill="both", expand=True)
        self.tabview.add("🤖  Simulador")
        self.tabview.add("📐  Analisis de angulos")
        self.tabview.add("📊  Datos del sensor")
        self.tabview.set("🤖  Simulador")

        # Tab simulador
        tab1 = self.tabview.tab("🤖  Simulador")
        self.fig = Figure(figsize=(8, 7), dpi=100, facecolor=C_PANEL)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#F8FAFF")
        self.canvas = FigureCanvasTkAgg(self.fig, master=tab1)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_click)

        # Tab analisis
        tab2 = self.tabview.tab("📐  Analisis de angulos")
        self.fig2 = Figure(figsize=(8, 7), dpi=100, facecolor=C_PANEL)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=tab2)
        self.canvas2.get_tk_widget().pack(fill="both", expand=True)

        # Tab datos del sensor (NUEVO)
        tab3 = self.tabview.tab("📊  Datos del sensor")
        self.fig3 = Figure(figsize=(8, 7), dpi=100, facecolor=C_PANEL)
        self.canvas3 = FigureCanvasTkAgg(self.fig3, master=tab3)
        self.canvas3.get_tk_widget().pack(fill="both", expand=True)

        self.tabview.configure(command=self._on_tab_change)

    def _section(self, parent, text):
        f = ctk.CTkFrame(parent, fg_color=C_BORDER, height=1, corner_radius=0)
        f.pack(fill="x", pady=(14, 2))
        ctk.CTkLabel(parent, text=text.upper(),
                     font=("Helvetica", 10, "bold"),
                     text_color=C_SUBTEXT).pack(anchor="w", pady=(2, 4))

    # ── Panel de entrada manual ──────────────────────────────────────
    def _build_manual_panel(self, parent):
        # Una fila por cada modo: cajas de entrada + boton "Ir"
        def add_row(label, hint_a, hint_b, mode_id):
            f = ctk.CTkFrame(parent, fg_color="transparent")
            f.pack(fill="x", pady=2)
            ctk.CTkLabel(f, text=label, width=60,
                         font=("Helvetica", 11),
                         text_color=C_TEXT).pack(side="left")
            ea = ctk.CTkEntry(f, placeholder_text=hint_a, width=70,
                              font=("Helvetica", 11),
                              border_color=C_BORDER)
            ea.pack(side="left", padx=2)
            eb = ctk.CTkEntry(f, placeholder_text=hint_b, width=70,
                              font=("Helvetica", 11),
                              border_color=C_BORDER)
            eb.pack(side="left", padx=2)
            ctk.CTkButton(f, text="Ir", width=36,
                          fg_color=C_ACCENT, hover_color="#1D4ED8",
                          font=("Helvetica", 11, "bold"),
                          command=lambda: self._send_manual(mode_id, ea, eb)
                          ).pack(side="left", padx=2)
            return ea, eb

        self.e_xy_a, self.e_xy_b = add_row("X,Y", "x (cm)", "y (cm)", "xy")
        self.e_t_a,  self.e_t_b  = add_row("θ₁,θ₂", "θ1 (°)", "θ2 (°)", "t1t2")
        self.e_s_a,  self.e_s_b  = add_row("S1,S2", "S1 0-180", "S2 0-180", "s1s2")

        self.manual_status = ctk.CTkLabel(parent, text="",
                                          font=("Courier", 10),
                                          text_color=C_SUBTEXT)
        self.manual_status.pack(anchor="w", pady=2)

    def _send_manual(self, mode, entry_a, entry_b):
        try:
            a = float(entry_a.get())
            b = float(entry_b.get())
        except ValueError:
            self.manual_status.configure(text="⚠ Valores invalidos",
                                         text_color=C_WARN)
            return

        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        s1, s2 = None, None

        if mode == "xy":
            sol = inverse_kinematics(base, L1, L2, (a, b), self.elbow_up.get())
            if sol is None:
                self.manual_status.configure(text="⚠ Punto inalcanzable",
                                             text_color=C_WARN)
                return
            s1, s2 = to_servo_angles(*sol)
            self.target = (a, b)

        elif mode == "t1t2":
            t1r = np.radians(a)
            t2r = np.radians(b)
            s1, s2 = to_servo_angles(t1r, t2r)
            _, end = forward_kinematics(base, L1, L2, t1r, t2r)
            self.target = end

        elif mode == "s1s2":
            s1 = int(np.clip(round(a), 0, 180))
            s2 = int(np.clip(round(b), 0, 180))
            t1r = np.radians(s1 - 90.0)
            t2r = np.radians(s2)
            _, end = forward_kinematics(base, L1, L2, t1r, t2r)
            self.target = end

        self.manual_status.configure(
            text=f"→ servo1={s1}°  servo2={s2}°",
            text_color=C_ACCENT2)
        self.send_to_arduino(s1, s2)
        self.redraw()

    # ── Panel de secuencias ──────────────────────────────────────────
    def _build_sequences_panel(self, parent):
        btn_cfg = dict(font=("Helvetica", 12), height=32,
                       fg_color=C_ACCENT, hover_color="#1D4ED8",
                       text_color="white", corner_radius=8)

        self.btn_fill = ctk.CTkButton(
            parent, text="▶  Barrido relleno",
            command=lambda: self._toggle_sequence("fill"), **btn_cfg)
        self.btn_fill.pack(fill="x", pady=3)

        self.btn_border = ctk.CTkButton(
            parent, text="▶  Barrido de orilla",
            command=lambda: self._toggle_sequence("border"), **btn_cfg)
        self.btn_border.pack(fill="x", pady=3)

        self.btn_corners = ctk.CTkButton(
            parent, text="▶  Esquinas + centro  📡",
            command=lambda: self._toggle_sequence("corners"), **btn_cfg)
        self.btn_corners.pack(fill="x", pady=3)

        # Medir en el punto actual del brazo (clic o coordenadas manuales)
        measure_cfg = dict(btn_cfg)
        measure_cfg.update(fg_color=C_ACCENT2, hover_color="#15803D")
        self.btn_measure = ctk.CTkButton(
            parent, text="📡  Medir aqui (promedio)",
            command=self._measure_current_point, **measure_cfg)
        self.btn_measure.pack(fill="x", pady=3)

        self.seq_status = ctk.CTkLabel(parent, text="",
                                       font=("Courier", 10),
                                       text_color=C_SUBTEXT)
        self.seq_status.pack(anchor="w", pady=2)

        # ── Parametros de medicion ──────────────────────────────────
        ctk.CTkFrame(parent, fg_color=C_BORDER, height=1,
                     corner_radius=0).pack(fill="x", pady=(10, 2))
        ctk.CTkLabel(parent, text="MEDICION VEML7700 (ESQUINAS)",
                     font=("Helvetica", 10, "bold"),
                     text_color=C_SUBTEXT).pack(anchor="w", pady=(2, 4))

        LabeledEntry(parent, "Seg. por punto",
                     self.scan_seconds).pack(fill="x", pady=3)
        LabeledEntry(parent, "Mediciones (cantidad)",
                     self.scan_samples).pack(fill="x", pady=3)
        LabeledEntry(parent, "ms entre mediciones",
                     self.scan_delay_ms).pack(fill="x", pady=3)

        ctk.CTkButton(parent, text="💾  Exportar CSV",
                      fg_color=C_ACCENT2, hover_color="#15803D",
                      font=("Helvetica", 11), height=28,
                      command=self._export_csv).pack(fill="x", pady=(6, 2))

        ctk.CTkButton(parent, text="🗑  Limpiar resultados",
                      fg_color=C_BORDER, text_color=C_TEXT,
                      hover_color="#CBD5E1",
                      font=("Helvetica", 11), height=28,
                      command=self._clear_results).pack(fill="x", pady=2)

        ctk.CTkLabel(parent, text="RESULTADOS",
                     font=("Helvetica", 10, "bold"),
                     text_color=C_SUBTEXT).pack(anchor="w", pady=(8, 2))
        self.results_box = ctk.CTkTextbox(
            parent, height=120, font=("Courier", 9),
            fg_color="#F8FAFF", border_color=C_BORDER,
            text_color=C_TEXT, state="disabled")
        self.results_box.pack(fill="x", pady=2)

    # ── Panel de conexion (comboboxes: Serial USB / WiFi TCP) ─────────
    def _build_serial_panel(self, parent):
        # Modo de conexion
        mode_row = ctk.CTkFrame(parent, fg_color="transparent")
        mode_row.pack(fill="x", pady=2)
        ctk.CTkLabel(mode_row, text="Modo:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        self.mode_combo = ctk.CTkComboBox(
            mode_row, values=["Serial", "WiFi"], variable=self.conn_mode,
            width=120, font=("Helvetica", 11), border_color=C_BORDER,
            state="readonly", command=lambda _=None: self._on_mode_change())
        self.mode_combo.pack(side="left", padx=2)

        # ── Sub-frame Serial ──────────────────────────────────────────
        self.serial_frame = ctk.CTkFrame(parent, fg_color="transparent")

        row1 = ctk.CTkFrame(self.serial_frame, fg_color="transparent")
        row1.pack(fill="x", pady=2)
        ctk.CTkLabel(row1, text="Puerto:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        self.port_combo = ctk.CTkComboBox(
            row1, values=self._available_ports(), variable=self.port_var,
            width=120, font=("Helvetica", 11), border_color=C_BORDER)
        self.port_combo.pack(side="left", padx=2)

        row2 = ctk.CTkFrame(self.serial_frame, fg_color="transparent")
        row2.pack(fill="x", pady=2)
        ctk.CTkLabel(row2, text="Baud:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        self.baud_combo = ctk.CTkComboBox(
            row2, values=["9600", "19200", "38400", "57600", "115200"],
            variable=self.baud_var, width=120, font=("Helvetica", 11),
            border_color=C_BORDER)
        self.baud_combo.pack(side="left", padx=2)

        if SERIAL_OK:
            ctk.CTkButton(self.serial_frame, text="Refrescar puertos",
                          fg_color=C_BORDER, text_color=C_TEXT,
                          hover_color="#CBD5E1", height=24,
                          font=("Helvetica", 10),
                          command=self._refresh_ports).pack(fill="x", pady=2)

        # ── Sub-frame WiFi ────────────────────────────────────────────
        self.wifi_frame = ctk.CTkFrame(parent, fg_color="transparent")

        rowi = ctk.CTkFrame(self.wifi_frame, fg_color="transparent")
        rowi.pack(fill="x", pady=2)
        ctk.CTkLabel(rowi, text="IP:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        self.ip_combo = ctk.CTkComboBox(
            rowi, values=self._recent_ips, variable=self.ip_var,
            width=120, font=("Helvetica", 11), border_color=C_BORDER)
        self.ip_combo.pack(side="left", padx=2)

        rowp = ctk.CTkFrame(self.wifi_frame, fg_color="transparent")
        rowp.pack(fill="x", pady=2)
        ctk.CTkLabel(rowp, text="Puerto:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        self.tcp_port_combo = ctk.CTkComboBox(
            rowp, values=["5000"], variable=self.tcp_port_var,
            width=120, font=("Helvetica", 11), border_color=C_BORDER)
        self.tcp_port_combo.pack(side="left", padx=2)

        ctk.CTkLabel(self.wifi_frame,
                     text="(IP que imprime el Arduino en el Monitor Serial)",
                     font=("Helvetica", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", pady=(0, 4))

        # Mostrar el sub-frame correspondiente al modo inicial
        self._on_mode_change()

        self.btn_conn = ctk.CTkButton(
            parent, text="Conectar",
            fg_color=C_ACCENT2, hover_color="#15803D",
            font=("Helvetica", 12, "bold"),
            command=self._toggle_connection)
        self.btn_conn.pack(fill="x", pady=4)

        self.conn_indicator = ctk.CTkLabel(
            parent, text="⬤  Desconectado",
            font=("Helvetica", 11, "bold"), text_color=C_WARN)
        self.conn_indicator.pack(anchor="w")

        self.cmd_lbl = ctk.CTkLabel(parent, text="",
                                    font=("Courier", 10),
                                    text_color=C_SUBTEXT)
        self.cmd_lbl.pack(anchor="w", pady=(4, 0))

    def _on_mode_change(self):
        """Muestra el sub-frame del modo activo y oculta el otro."""
        wifi = self.conn_mode.get() == "WiFi"
        show, hide = (self.wifi_frame, self.serial_frame) if wifi \
            else (self.serial_frame, self.wifi_frame)
        hide.pack_forget()
        # Mantener el sub-frame por encima del boton Conectar (si ya existe)
        if hasattr(self, "btn_conn"):
            show.pack(fill="x", before=self.btn_conn)
        else:
            show.pack(fill="x")

    def _available_ports(self):
        if not SERIAL_OK:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        ports = self._available_ports()
        self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ================================================================== #
    @property
    def base(self):
        return (self.ancho.get() / 2.0, -self.a.get())

    @property
    def reach(self):
        return self.L1.get() + self.L2.get()

    def corner_dist(self):
        w, h = self.ancho.get(), self.alto.get()
        bx, by = self.base
        corners = [(0, 0), (w, 0), (w, h), (0, h)]
        return max(np.hypot(cx - bx, cy - by) for cx, cy in corners)

    # ================================================================== #
    def _on_param_change(self):
        self.redraw()
        self._compute_sweep_angles()
        self._draw_angle_chart()
        self._draw_sensor_chart()

    def _on_tab_change(self):
        self._draw_angle_chart()
        self._draw_sensor_chart()

    # ================================================================== #
    #  SIMULADOR
    # ================================================================== #
    def redraw(self):
        if self.anim_running:
            return
        ax = self.ax
        ax.clear()
        ax.set_facecolor("#F8FAFF")

        w, h = self.ancho.get(), self.alto.get()
        a = self.a.get()
        L1, L2 = self.L1.get(), self.L2.get()
        base = self.base

        ws = patches.Circle(base, self.reach,
                            facecolor=C_ARM1, alpha=0.06,
                            edgecolor=C_ARM1, lw=1.5, ls="--")
        ax.add_patch(ws)

        scr = patches.FancyBboxPatch((0, 0), w, h,
                                     boxstyle="round,pad=0.1",
                                     facecolor=C_SCREEN, alpha=0.5,
                                     edgecolor=C_ACCENT, lw=1.8)
        ax.add_patch(scr)
        ax.text(w / 2, h + 0.5, f"Pantalla  {w:.1f} × {h:.1f} cm",
                ha="center", color=C_ACCENT, fontsize=9, fontweight="bold")

        ax.plot(*base, "s", color=C_ARM2, ms=11, zorder=5)
        ax.text(base[0] + 0.3, base[1] - 0.5,
                f"base ({base[0]:.1f}, {base[1]:.1f})",
                fontsize=8, color=C_ARM2)

        if self.target is not None:
            sol = inverse_kinematics(base, L1, L2, self.target,
                                     self.elbow_up.get())
            if sol:
                t1, t2 = sol
                elbow, end = forward_kinematics(base, L1, L2, t1, t2)
                ax.plot([base[0], elbow[0]], [base[1], elbow[1]],
                        "-", color=C_ARM1, lw=5, solid_capstyle="round", zorder=4)
                ax.plot([elbow[0], end[0]], [elbow[1], end[1]],
                        "-", color=C_ARM2, lw=5, solid_capstyle="round", zorder=4)
                ax.plot(*elbow, "o", color=C_ARM1, ms=9, zorder=5)
                ax.plot(*end, "o", color=C_TARGET, ms=10, zorder=6,
                        markeredgecolor="white", markeredgewidth=1.5)
                ax.plot(*self.target, "+", color=C_TEXT, ms=10, mew=2, zorder=7)

                s1, s2 = to_servo_angles(t1, t2)
                self.angle_lbl.configure(
                    text=(f"x={self.target[0]:.2f}  y={self.target[1]:.2f}\n"
                          f"θ1 = {np.degrees(t1):+7.2f}°\n"
                          f"θ2 = {np.degrees(t2):+7.2f}°\n"
                          f"servo1 = {s1}°   servo2 = {s2}°"),
                    text_color=C_TEXT)
            else:
                ax.plot(*self.target, "x", color=C_WARN, ms=13, mew=3, zorder=7)
                self.angle_lbl.configure(text="⚠ Punto inalcanzable",
                                         text_color=C_WARN)

        d = self.corner_dist()
        ok = self.reach >= d
        mg = self.reach - d
        self.cond_lbl.configure(
            text=(f"L1+L2 = {self.reach:.2f} cm\n"
                  f"d_max = {d:.2f} cm\n"
                  f"margen = {mg:+.2f} cm\n"
                  f"{'✓ Alcanza esquinas' if ok else '✗ NO alcanza'}"),
            text_color=C_ACCENT2 if ok else C_WARN)

        lim = max(w, h) + self.reach
        ax.set_xlim(base[0] - lim / 2, base[0] + lim / 2)
        ax.set_ylim(-a - 4, base[1] + self.reach + 2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.12, color="#CBD5E1")
        ax.set_xlabel("x (cm)", fontsize=9, color=C_SUBTEXT)
        ax.set_ylabel("y (cm)", fontsize=9, color=C_SUBTEXT)
        ax.set_title("Brazo 2-DOF - clic para resolver IK",
                     fontsize=11, color=C_TEXT, pad=10)
        ax.tick_params(colors=C_SUBTEXT, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor(C_BORDER)
        self.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax or self.anim_running or event.xdata is None:
            return
        self.target = (event.xdata, event.ydata)
        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        sol = inverse_kinematics(base, L1, L2, self.target, self.elbow_up.get())
        if sol:
            s1, s2 = to_servo_angles(*sol)
            self.send_to_arduino(s1, s2)
        self.redraw()

    # ================================================================== #
    #  SECUENCIAS
    # ================================================================== #
    def _toggle_sequence(self, name):
        buttons = {"fill": self.btn_fill,
                   "border": self.btn_border,
                   "corners": self.btn_corners}
        labels = {"fill": "▶  Barrido relleno",
                  "border": "▶  Barrido de orilla",
                  "corners": "▶  Esquinas + centro  📡"}

        self._moving = False   # cancela cualquier movimiento suave en curso

        if self.anim_running:
            self._stop_all()
            for n, b in buttons.items():
                b.configure(text=labels[n], fg_color=C_ACCENT)
            return

        path = self._make_path(name)
        if not path:
            messagebox.showwarning("Sin puntos",
                                   "Ningun punto alcanzable con estos parametros.")
            return

        for n, b in buttons.items():
            if n == name:
                b.configure(text="⏹  Detener", fg_color=C_WARN)
            else:
                b.configure(state="disabled")
        self.btn_measure.configure(state="disabled")

        if name == "corners":
            self._start_corners_sequence(path)
        else:
            self._start_anim_sequence(path)

    def _make_path(self, name):
        w, h = self.ancho.get(), self.alto.get()
        L1, L2 = self.L1.get(), self.L2.get()
        base = self.base
        eu = self.elbow_up.get()
        off = 0.5
        pts = []

        if name == "fill":
            n_rows, n_cols = 6, 35
            for i, y in enumerate(np.linspace(off, h - off, n_rows)):
                xs = np.linspace(off, w - off, n_cols)
                if i % 2:
                    xs = xs[::-1]
                pts.extend((x, y) for x in xs)

        elif name == "border":
            n = 40
            bot = [(x, off) for x in np.linspace(off, w - off, n)]
            rgt = [(w - off, y) for y in np.linspace(off, h - off, n)[1:]]
            top = [(x, h - off) for x in np.linspace(w - off, off, n)[1:]]
            lft = [(off, y) for y in np.linspace(h - off, off, n)[1:]]
            pts = bot + rgt + top + lft

        elif name == "corners":
            # Esquinas REALES de la pantalla (sin offset). Si alguna no es
            # alcanzable, se acerca lo mas posible hacia el centro.
            raw = [(0, 0), (w, 0), (w, h), (0, h), (w / 2, h / 2)]
            pts = [self._nearest_reachable(c) for c in raw]
            return [p for p in pts if p is not None]

        return [(x, y) for x, y in pts
                if inverse_kinematics(base, L1, L2, (x, y), eu) is not None]

    def _nearest_reachable(self, pt):
        """Devuelve pt si es alcanzable; si no, lo acerca al centro de la
        pantalla en pasos pequenos hasta que el IK tenga solucion (offset
        minimo). Devuelve None si nada es alcanzable."""
        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        eu = self.elbow_up.get()
        if inverse_kinematics(base, L1, L2, pt, eu) is not None:
            return pt
        cx, cy = self.ancho.get() / 2.0, self.alto.get() / 2.0
        for f in np.linspace(0.02, 0.6, 30):
            nx = pt[0] + f * (cx - pt[0])
            ny = pt[1] + f * (cy - pt[1])
            if inverse_kinematics(base, L1, L2, (nx, ny), eu) is not None:
                return (nx, ny)
        return None

    # ── Secuencia esquinas con medicion ─────────────────────────────
    def _start_corners_sequence(self, waypoints):
        self.scan_results = []
        self.anim_running = True
        self._seq_stop.clear()
        self._setup_anim_canvas()
        self._last_s = (-1, -1)
        t = threading.Thread(
            target=self._corners_measure_thread,
            args=(waypoints,), daemon=True)
        t.start()

    def _corners_measure_thread(self, waypoints):
        NAMES = ["Inf-Izq", "Inf-Der", "Sup-Der", "Sup-Izq", "Centro"]
        L1, L2 = self.L1.get(), self.L2.get()
        base = self.base
        n_samp = int(self.scan_samples.get())
        n_secs = int(self.scan_seconds.get())
        delay_ms = int(self.scan_delay_ms.get())

        try:
            for i, dest in enumerate(waypoints):
                if self._seq_stop.is_set():
                    break
                label = NAMES[i] if i < len(NAMES) else f"Punto {i+1}"
                sol = inverse_kinematics(base, L1, L2, dest, self.elbow_up.get())
                if sol is None:
                    continue

                # Recupera la conexion si el WiFi se cayo entre puntos
                self._ensure_connected()

                if i > 0:
                    orig = waypoints[i - 1]
                    self._anim_transit(orig, dest, label, L1, L2, base)
                    if self._seq_stop.is_set():
                        break

                s1, s2 = to_servo_angles(*sol)
                self.send_to_arduino(s1, s2)
                self._last_s = (s1, s2)
                self.after(0, lambda pt=dest, lb=label, s=sol:
                           self._corners_update_canvas(pt, lb, s))

                lux, white, muestras = self._do_scan(
                    label, dest, n_samp, n_secs, delay_ms)

                if lux is not None and not self._seq_stop.is_set():
                    result = {"label": label,
                              "x": dest[0], "y": dest[1],
                              "iluminancia": lux,
                              "luminancia": lux / OMEGA_SR,
                              "muestras": muestras}
                    self.scan_results.append(result)
                    self.session_results.append(result)
                    self._publish_thingspeak(result)
                    self.after(0, self._refresh_results_box)
                    self.after(0, self._draw_sensor_chart)
        finally:
            # Pase lo que pase, desbloquear la UI
            self.after(0, self._stop_all)

    # ── Medicion en el punto actual del brazo ────────────────────────
    def _measure_current_point(self):
        # Si ya hay algo corriendo, este boton actua como "Detener"
        if self.anim_running:
            self._stop_all()
            return
        if self.target is None:
            messagebox.showinfo(
                "Sin posicion",
                "Primero mueve el brazo a un punto: haz clic en el "
                "simulador o usa la entrada manual de coordenadas.")
            return
        if not (self.transport and self.transport.is_open):
            messagebox.showwarning("Sin conexion",
                                   "Conecta el Arduino primero.")
            return

        self.anim_running = True
        self._seq_stop.clear()
        self.btn_measure.configure(text="⏹  Detener", fg_color=C_WARN)
        for b in (self.btn_fill, self.btn_border, self.btn_corners):
            b.configure(state="disabled")

        t = threading.Thread(target=self._measure_current_thread, daemon=True)
        t.start()

    def _measure_current_thread(self):
        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        pt = self.target
        n_samp = int(self.scan_samples.get())
        n_secs = int(self.scan_seconds.get())
        delay_ms = int(self.scan_delay_ms.get())

        try:
            # Reasegura que el brazo este en el punto antes de medir
            sol = inverse_kinematics(base, L1, L2, pt, self.elbow_up.get())
            if sol is not None and self.transport and self.transport.is_open:
                s1, s2 = to_servo_angles(*sol)
                self.send_to_arduino(s1, s2)
                time.sleep(0.4)   # deja que el servo llegue antes de medir

            label = f"({pt[0]:.1f},{pt[1]:.1f})"
            lux, white, muestras = self._do_scan(
                label, pt, n_samp, n_secs, delay_ms)

            if lux is not None and not self._seq_stop.is_set():
                result = {"label": label,
                          "x": pt[0], "y": pt[1],
                          "iluminancia": lux,
                          "luminancia": lux / OMEGA_SR,
                          "muestras": muestras}
                self.scan_results.append(result)
                self.session_results.append(result)
                self._publish_thingspeak(result)
                self.after(0, self._refresh_results_box)
                self.after(0, self._draw_sensor_chart)
        finally:
            self.after(0, self._stop_all)

    def _anim_transit(self, orig, dest, label, L1, L2, base):
        N_T = 40
        done = threading.Event()
        count = [0]

        transit_pts = []
        for k in range(N_T):
            t = k / (N_T - 1)
            t_ease = 0.5 - 0.5 * np.cos(np.pi * t)
            ix = orig[0] + t_ease * (dest[0] - orig[0])
            iy = orig[1] + t_ease * (dest[1] - orig[1])
            transit_pts.append((ix, iy))

        def update(frame):
            count[0] += 1
            pt = transit_pts[frame]
            sol = inverse_kinematics(base, L1, L2, pt, self.elbow_up.get())
            if sol:
                t1, t2 = sol
                elbow, end = forward_kinematics(base, L1, L2, t1, t2)
                self.link1.set_data([base[0], elbow[0]], [base[1], elbow[1]])
                self.link2.set_data([elbow[0], end[0]], [elbow[1], end[1]])
                self.joint_e.set_data([elbow[0]], [elbow[1]])
                self.joint_t.set_data([end[0]], [end[1]])
                self.trail_line.set_data([], [])
                s1, s2 = to_servo_angles(t1, t2)
                if (s1, s2) != self._last_s:
                    self.send_to_arduino(s1, s2)
                    self._last_s = (s1, s2)
                self.seq_status.configure(
                    text=f"→ transito  {label}  ({pt[0]:.1f}, {pt[1]:.1f})",
                    text_color=C_ACCENT)
            if count[0] >= N_T:
                done.set()
            return self.link1, self.link2, self.joint_e, self.joint_t, self.trail_line

        def launch():
            self.anim = animation.FuncAnimation(
                self.fig, update, frames=N_T,
                interval=50, blit=False, repeat=False)
            self.canvas.draw_idle()

        self.after(0, launch)
        done.wait(timeout=N_T * 0.06 + 2.0)

    def _corners_update_canvas(self, pt, label, sol):
        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        t1, t2 = sol
        elbow, end = forward_kinematics(base, L1, L2, t1, t2)
        s1, s2 = to_servo_angles(t1, t2)
        self.link1.set_data([base[0], elbow[0]], [base[1], elbow[1]])
        self.link2.set_data([elbow[0], end[0]], [elbow[1], end[1]])
        self.joint_e.set_data([elbow[0]], [elbow[1]])
        self.joint_t.set_data([end[0]], [end[1]])
        self.trail_line.set_data([], [])
        self.canvas.draw_idle()
        self.seq_status.configure(
            text=f"📡 Midiendo en {label}  ({pt[0]:.1f}, {pt[1]:.1f})",
            text_color=C_ACCENT2)
        self.angle_lbl.configure(
            text=(f"x={pt[0]:.2f}  y={pt[1]:.2f}\n"
                  f"θ1 = {np.degrees(t1):+7.2f}°\n"
                  f"θ2 = {np.degrees(t2):+7.2f}°\n"
                  f"servo1 = {s1}°   servo2 = {s2}°"),
            text_color=C_TEXT)

    def _do_scan(self, label, pt, n_samp, n_secs, delay_ms=120):
        """Devuelve (lux_prom, white_prom, muestras) donde muestras es la
        lista de lecturas crudas de iluminancia (lux) que mando el Arduino."""
        if not (self.transport and self.transport.is_open):
            for s in range(n_secs, 0, -1):
                if self._seq_stop.is_set():
                    return None, None, []
                self.after(0, lambda sec=s, lb=label:
                           self.seq_status.configure(
                               text=f"📡 {lb} - sin serial, esperando {sec}s",
                               text_color=C_WARN))
                time.sleep(1.0)
            return None, None, []

        with self._serial_lock:
            self.transport.reset_input_buffer()

        # tiempo estimado: n muestras * (delay + ~10ms de lectura), con piso n_secs
        estimated = max(n_secs, n_samp * (delay_ms + 10) / 1000.0)
        t_start = time.time()

        cmd = f"SCAN:{n_samp},{delay_ms}\n"
        try:
            with self._serial_lock:
                self.transport.write(cmd.encode())
        except Exception as e:
            self.after(0, lambda: self.cmd_lbl.configure(text=f"Error: {e}"))
            return None, None, []

        def tick():
            elapsed = time.time() - t_start
            remaining = max(0, estimated - elapsed)
            self.seq_status.configure(
                text=f"📡 {label} - midiendo... {remaining:.1f}s",
                text_color=C_ACCENT2)

        muestras = []
        deadline = time.time() + estimated + 10.0
        while time.time() < deadline:
            if self._seq_stop.is_set():
                return None, None, muestras
            self.after(0, tick)
            time.sleep(0.2)
            try:
                # Drena todas las lineas disponibles este ciclo
                got = []
                with self._serial_lock:
                    for _ in range(300):
                        if not self.transport.in_waiting:
                            break
                        ln = self.transport.readline().decode(
                            errors="ignore").strip()
                        if not ln:
                            break
                        got.append(ln)
                for line in got:
                    if line.startswith("SAMPLE:"):
                        try:
                            muestras.append(float(line[len("SAMPLE:"):]))
                        except ValueError:
                            pass
                    elif line.startswith("SCAN_RESULT:"):
                        parts = line[len("SCAN_RESULT:"):].split(",")
                        if len(parts) == 2:
                            lux = float(parts[0])
                            white = float(parts[1])
                            self.after(0, lambda lb=label, lx=lux, wh=white,
                                       k=len(muestras):
                                       self.seq_status.configure(
                                           text=f"✓ {lb}  Lux={lx:.2f}  ({k} muestras)",
                                           text_color=C_ACCENT2))
                            return lux, white, muestras
            except Exception:
                pass

        self.after(0, lambda lb=label:
                   self.seq_status.configure(
                       text=f"⚠ {lb} - timeout esperando SCAN_RESULT",
                       text_color=C_WARN))
        return None, None, muestras

    def _refresh_results_box(self):
        self.results_box.configure(state="normal")
        self.results_box.delete("1.0", "end")
        header = f"{'Punto':<10} {'Ilum(lux)':>10} {'Lum(cd/m2)':>11}\n"
        self.results_box.insert("end", header)
        self.results_box.insert("end", "-" * 33 + "\n")
        for r in self.scan_results:
            if r.get("iluminancia") is not None:
                line = (f"{r['label']:<10} {r['iluminancia']:>10.2f} "
                        f"{r['luminancia']:>11.2f}\n")
            else:
                line = f"{r['label']:<10} {'N/A':>10} {'N/A':>11}\n"
            self.results_box.insert("end", line)
        self.results_box.configure(state="disabled")

    def _export_csv(self):
        # Acumulativo por sesion: usa todo lo medido (session_results) y un
        # mismo archivo durante la sesion. Formato largo: una fila por cada
        # muestra cruda, mas el promedio del punto.
        if not self.session_results:
            messagebox.showinfo("Sin datos", "Aun no hay resultados.")
            return
        if self._csv_session_file is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._csv_session_file = os.path.abspath(f"scan_sesion_{ts}.csv")
        filename = self._csv_session_file
        try:
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "label", "x", "y", "muestra",
                    "iluminancia_lux", "luminancia_cd_m2",
                    "iluminancia_prom_lux", "luminancia_prom_cd_m2"])
                for r in self.session_results:
                    if r.get("iluminancia") is None:
                        continue
                    prom_i = r["iluminancia"]
                    prom_l = r["luminancia"]
                    muestras = r.get("muestras") or []
                    if muestras:
                        for i, lx in enumerate(muestras, start=1):
                            writer.writerow([
                                r["label"], r["x"], r["y"], i,
                                f"{lx:.4f}", f"{lx / OMEGA_SR:.4f}",
                                f"{prom_i:.4f}", f"{prom_l:.4f}"])
                    else:
                        # sin muestras crudas (firmware viejo): solo promedio
                        writer.writerow([
                            r["label"], r["x"], r["y"], "",
                            "", "", f"{prom_i:.4f}", f"{prom_l:.4f}"])
            messagebox.showinfo("Exportado",
                                f"Guardado ({len(self.session_results)} puntos):\n{filename}")
            try:
                os.startfile(filename)   # abre el archivo (Windows)
            except Exception:
                pass
        except Exception as e:
            messagebox.showerror("Error al exportar", str(e))

    def _clear_results(self):
        self.scan_results = []
        self._refresh_results_box()
        self._draw_sensor_chart()

    # ── Secuencia animada (fill / border) ───────────────────────────
    def _start_anim_sequence(self, path):
        L1, L2 = self.L1.get(), self.L2.get()
        base = self.base
        self.anim_running = True
        self._seq_stop.clear()
        self._setup_anim_canvas()

        trail_x, trail_y = [], []
        self._last_s = (-1, -1)
        total_frames = len(path)
        self._anim_frame_count = 0

        def update(frame):
            self._anim_frame_count += 1
            pt = path[frame]
            sol = inverse_kinematics(base, L1, L2, pt, self.elbow_up.get())
            if sol:
                t1, t2 = sol
                elbow, end = forward_kinematics(base, L1, L2, t1, t2)
                self.link1.set_data([base[0], elbow[0]], [base[1], elbow[1]])
                self.link2.set_data([elbow[0], end[0]], [elbow[1], end[1]])
                self.joint_e.set_data([elbow[0]], [elbow[1]])
                self.joint_t.set_data([end[0]], [end[1]])
                trail_x.append(end[0]); trail_y.append(end[1])
                self.trail_line.set_data(trail_x, trail_y)
                s1, s2 = to_servo_angles(t1, t2)
                if (s1, s2) != self._last_s:
                    self.send_to_arduino(s1, s2)
                    self._last_s = (s1, s2)
                self.seq_status.configure(
                    text=f"({pt[0]:.1f}, {pt[1]:.1f})  s1={s1}° s2={s2}°",
                    text_color=C_ACCENT)
            if self._anim_frame_count >= total_frames:
                self.after(0, self._stop_all)
            return self.link1, self.link2, self.joint_e, self.joint_t, self.trail_line

        self.anim = animation.FuncAnimation(
            self.fig, update, frames=total_frames,
            interval=50, blit=False, repeat=False)
        self.canvas.draw_idle()

    def _setup_anim_canvas(self):
        ax = self.ax
        ax.clear()
        ax.set_facecolor("#F8FAFF")
        w, h = self.ancho.get(), self.alto.get()
        a = self.a.get()
        base = self.base

        ws = patches.Circle(base, self.reach,
                            facecolor=C_ARM1, alpha=0.06,
                            edgecolor=C_ARM1, lw=1.5, ls="--")
        ax.add_patch(ws)
        scr = patches.FancyBboxPatch((0, 0), w, h,
                                     boxstyle="round,pad=0.1",
                                     facecolor=C_SCREEN, alpha=0.5,
                                     edgecolor=C_ACCENT, lw=1.8)
        ax.add_patch(scr)
        ax.plot(*base, "s", color=C_ARM2, ms=11, zorder=5)

        (self.link1,) = ax.plot([], [], "-", color=C_ARM1, lw=5,
                                solid_capstyle="round")
        (self.link2,) = ax.plot([], [], "-", color=C_ARM2, lw=5,
                                solid_capstyle="round")
        (self.joint_e,) = ax.plot([], [], "o", color=C_ARM1, ms=9)
        (self.joint_t,) = ax.plot([], [], "o", color=C_TARGET, ms=10)
        (self.trail_line,) = ax.plot([], [], "-", color=C_TRAIL,
                                     lw=1.2, alpha=0.5)

        lim = max(w, h) + self.reach
        ax.set_xlim(base[0] - lim / 2, base[0] + lim / 2)
        ax.set_ylim(-a - 4, base[1] + self.reach + 2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.12, color="#CBD5E1")
        ax.set_xlabel("x (cm)", fontsize=9, color=C_SUBTEXT)
        ax.set_ylabel("y (cm)", fontsize=9, color=C_SUBTEXT)
        ax.tick_params(colors=C_SUBTEXT, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor(C_BORDER)

    def _stop_all(self):
        # A prueba de fallos: el reset de estado/botones SIEMPRE debe correr,
        # aunque detener la animacion lance (p.ej. event_source ya None).
        self._seq_stop.set()
        try:
            if self.anim:
                if self.anim.event_source:
                    self.anim.event_source.stop()
        except Exception:
            pass
        finally:
            self.anim = None

        self.anim_running = False
        for btn, lbl in [(self.btn_fill,    "▶  Barrido relleno"),
                         (self.btn_border,  "▶  Barrido de orilla"),
                         (self.btn_corners, "▶  Esquinas + centro  📡")]:
            btn.configure(text=lbl, fg_color=C_ACCENT, state="normal")
        self.btn_measure.configure(text="📡  Medir aqui (promedio)",
                                   fg_color=C_ACCENT2, state="normal")
        self.seq_status.configure(text="")
        self.redraw()

    # ================================================================== #
    #  SERIAL
    # ================================================================== #
    def _toggle_connection(self):
        if self.transport and self.transport.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if self.conn_mode.get() == "WiFi":
            self._connect_wifi()
        else:
            self._connect_serial()

    def _connect_serial(self):
        if not SERIAL_OK:
            messagebox.showerror("pyserial no instalado",
                                 "pip install pyserial")
            return
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("Baud invalido", "Introduce un baud valido.")
            return
        if not port:
            messagebox.showwarning("Sin puerto",
                                   "Escribe el nombre del puerto.")
            return
        try:
            self.transport = SerialTransport(port, baud)
            self._last_conn = ("Serial", port, baud)
            self._upd_conn(True)
        except Exception as e:
            messagebox.showerror("Error de conexion", str(e))

    def _connect_wifi(self):
        host = self.ip_var.get().strip()
        if not host:
            messagebox.showwarning("Sin IP", "Escribe la IP del Arduino.")
            return
        try:
            tcp_port = int(self.tcp_port_var.get())
        except ValueError:
            messagebox.showwarning("Puerto invalido",
                                   "Introduce un puerto TCP valido.")
            return
        try:
            self.transport = TcpTransport(host, tcp_port)
            self._last_conn = ("WiFi", host, tcp_port)
            self._remember_ip(host)
            self._upd_conn(True)
        except (OSError, socket.timeout) as e:
            messagebox.showerror("Error de conexion WiFi",
                                 f"No se pudo conectar a {host}:{tcp_port}\n{e}")

    def _reconnect_transport(self):
        """Reabre el transporte con los ultimos parametros usados.
        Devuelve True si quedo conectado. Pensado para recuperarse de
        caidas intermitentes de WiFi durante una rutina."""
        if not self._last_conn:
            return False
        with self._serial_lock:
            if self.transport:
                try:
                    self.transport.close()
                except Exception:
                    pass
                self.transport = None
            try:
                kind = self._last_conn[0]
                if kind == "Serial":
                    _, port, baud = self._last_conn
                    self.transport = SerialTransport(port, baud)
                else:
                    _, host, port = self._last_conn
                    self.transport = TcpTransport(host, port)
                ok = True
            except Exception:
                self.transport = None
                ok = False
        self.after(0, lambda v=ok: self._upd_conn(v))
        return ok

    def _ensure_connected(self, retries=3):
        """Garantiza conexion antes de medir; reintenta si se cayo el WiFi.
        Se llama desde los hilos de medicion."""
        if self.transport and self.transport.is_open:
            return True
        self.after(0, lambda: self.seq_status.configure(
            text="🔌 conexion perdida, reconectando...", text_color=C_WARN))
        for _ in range(retries):
            if self._seq_stop.is_set():
                return False
            if self._reconnect_transport():
                return True
            time.sleep(1.0)
        return False

    def _remember_ip(self, ip):
        if ip not in self._recent_ips:
            self._recent_ips.insert(0, ip)
            self._recent_ips = self._recent_ips[:8]
            if hasattr(self, "ip_combo"):
                self.ip_combo.configure(values=self._recent_ips)

    def _disconnect(self):
        if self.transport:
            try:
                self.transport.close()
            except Exception:
                pass
            self.transport = None
        self._upd_conn(False)

    def _upd_conn(self, ok):
        if ok:
            self.conn_indicator.configure(text="⬤  Conectado",
                                          text_color=C_ACCENT2)
            self.btn_conn.configure(text="Desconectar", fg_color=C_WARN,
                                    hover_color="#B91C1C")
        else:
            self.conn_indicator.configure(text="⬤  Desconectado",
                                          text_color=C_WARN)
            self.btn_conn.configure(text="Conectar", fg_color=C_ACCENT2,
                                    hover_color="#15803D")

    def send_to_arduino(self, s1, s2):
        if not (self.transport and self.transport.is_open):
            return
        cmd = f"s1:{s1},s2:{s2}\n"
        try:
            with self._serial_lock:
                self.transport.write(cmd.encode())
            self.cmd_lbl.configure(text=f"↑ {cmd.strip()}")
        except Exception as e:
            self.cmd_lbl.configure(text=f"Error: {e}")

    # ================================================================== #
    #  THINGSPEAK
    # ================================================================== #
    def _build_thingspeak_panel(self, parent):
        ctk.CTkCheckBox(parent, text="Publicar a ThingSpeak",
                        variable=self.ts_enable,
                        checkmark_color="white",
                        fg_color=C_ACCENT).pack(anchor="w", pady=4)

        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=2)
        ctk.CTkLabel(row, text="API Key:", width=60,
                     font=("Helvetica", 11),
                     text_color=C_TEXT).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.ts_key_var,
                     width=160, font=("Helvetica", 11),
                     border_color=C_BORDER,
                     placeholder_text="Write API Key").pack(side="left", padx=2)

        ctk.CTkLabel(parent,
                     text="Campos: 1=Iluminancia  2=Luminancia  3=X  4=Y",
                     font=("Helvetica", 9), text_color=C_SUBTEXT
                     ).pack(anchor="w", pady=(0, 4))

    def _publish_thingspeak(self, result):
        """Sube un resultado de escaneo a ThingSpeak en un hilo daemon.
        No bloquea Tk y degrada silenciosamente si no hay red."""
        key = self.ts_key_var.get().strip()
        if not key or not self.ts_enable.get():
            return
        if result.get("iluminancia") is None:
            return
        url = "https://api.thingspeak.com/update?" + urllib.parse.urlencode({
            "api_key": key,
            "field1": result["iluminancia"],
            "field2": result["luminancia"],
            "field3": result["x"],
            "field4": result["y"],
        })

        def worker():
            try:
                with urllib.request.urlopen(url, timeout=5) as resp:
                    entry = resp.read().decode(errors="ignore").strip()
                msg = (f"TS ok #{entry}" if entry not in ("", "0")
                       else "TS rate-limited (espera 15s)")
            except Exception as e:
                msg = f"TS offline: {e}"
            self.after(0, lambda m=msg: self.cmd_lbl.configure(text=m))

        threading.Thread(target=worker, daemon=True).start()

    # ================================================================== #
    #  ANALISIS DE ANGULOS
    # ================================================================== #
    def _compute_sweep_angles(self):
        L1, L2, base = self.L1.get(), self.L2.get(), self.base
        w, h = self.ancho.get(), self.alto.get()
        off = 0.5
        n_r, n_c = 5, 30
        pts = []
        for i, y in enumerate(np.linspace(off, h - off, n_r)):
            xs = np.linspace(off, w - off, n_c)
            if i % 2:
                xs = xs[::-1]
            pts.extend((x, y) for x in xs)

        records = []
        for idx, pt in enumerate(pts):
            sol = inverse_kinematics(base, L1, L2, pt, self.elbow_up.get())
            if sol:
                t1, t2 = sol
                records.append({"frame": idx, "t1": np.degrees(t1),
                                "t2": np.degrees(t2)})
        self._sweep_angles = records

    def _draw_angle_chart(self):
        if not self._sweep_angles:
            return
        data = self._sweep_angles
        frames = np.array([d["frame"] for d in data])
        t1s = np.array([d["t1"] for d in data])
        t2s = np.array([d["t2"] for d in data])
        # Region permitida del angulo articular por servo segun su offset:
        #   s = theta + offset  debe quedar en [0, 180]  ->  theta in [-offset, 180-offset]
        lo1, hi1 = -S1_OFFSET, 180.0 - S1_OFFSET   # theta1: [-3, 177]
        lo2, hi2 = -S2_OFFSET, 180.0 - S2_OFFSET   # theta2: [-93, 87]

        self.fig2.clear()
        self.fig2.set_facecolor(C_PANEL)
        gs = self.fig2.add_gridspec(3, 1, height_ratios=[5, 5, 2],
                                    hspace=0.55, left=0.10, right=0.97,
                                    top=0.93, bottom=0.06)
        ax1 = self.fig2.add_subplot(gs[0])
        ax2 = self.fig2.add_subplot(gs[1])
        ax_s = self.fig2.add_subplot(gs[2])
        ax_s.axis("off")

        for ax in (ax1, ax2):
            ax.set_facecolor("#F8FAFF")
            for spine in ax.spines.values():
                spine.set_edgecolor(C_BORDER)
            ax.tick_params(colors=C_SUBTEXT, labelsize=8)
            ax.yaxis.label.set_color(C_SUBTEXT)

        def plot_angle(ax, frames, angles, color, label, lo, hi):
            n = len(frames)
            # Zonas prohibidas (fuera del rango permitido del servo)
            ax.axhspan(hi, max(angles.max() + 10, hi + 5),
                       color="#FEE2E2", alpha=0.6, zorder=0)
            ax.axhspan(min(angles.min() - 10, lo - 5), lo,
                       color="#FEE2E2", alpha=0.6, zorder=0)
            ax.axhline(hi, color=C_WARN, lw=1.4, ls="--", zorder=1,
                       label=f"limite servo [{lo:.0f}°, {hi:.0f}°]")
            ax.axhline(lo, color=C_WARN, lw=1.4, ls="--", zorder=1)
            for i in range(n - 1):
                viol = (angles[i] < lo or angles[i] > hi
                        or angles[i + 1] < lo or angles[i + 1] > hi)
                c = C_WARN if viol else color
                ax.plot(frames[i:i+2], angles[i:i+2], "-",
                        color=c, lw=2.2, zorder=3)
            vm = (angles < lo) | (angles > hi)
            if vm.any():
                ax.scatter(frames[vm], angles[vm],
                           color=C_WARN, s=20, zorder=5,
                           label=f"Violacion: {vm.sum()} pts")
            im = np.argmax(np.abs(angles))
            ax.annotate(f"{angles[im]:+.1f}°",
                        xy=(frames[im], angles[im]),
                        xytext=(frames[im] + max(1, n * 0.03),
                                angles[im] + 4 * np.sign(angles[im])),
                        fontsize=8, color=C_TEXT,
                        arrowprops=dict(arrowstyle="-",
                                        color="#94A3B8", lw=0.8))
            ax.set_ylabel("Angulo (°)", fontsize=9)
            ax.set_title(f"{label}  [{angles.min():+.1f}°, {angles.max():+.1f}°]",
                         fontsize=10, pad=5, color=C_TEXT)
            ax.set_xlim(frames[0] - 1, frames[-1] + 1)
            ax.grid(alpha=0.15, color=C_BORDER)
            ax.legend(fontsize=8, loc="upper right",
                      framealpha=0.9, edgecolor=C_BORDER)

        plot_angle(ax1, frames, t1s, C_ARM1, "θ1  Servo hombro", lo1, hi1)
        plot_angle(ax2, frames, t2s, C_ARM2, "θ2  Servo codo", lo2, hi2)
        ax1.set_xticklabels([])
        ax2.set_xlabel("Indice de punto en el barrido",
                       fontsize=9, color=C_SUBTEXT)

        viol1 = int(np.sum((t1s < lo1) | (t1s > hi1)))
        viol2 = int(np.sum((t2s < lo2) | (t2s > hi2)))
        ok1 = "✓ OK" if viol1 == 0 else f"✗ {viol1} pts"
        ok2 = "✓ OK" if viol2 == 0 else f"✗ {viol2} pts"
        rows = [
            ["θ1 hombro", f"{t1s.min():+.1f}°", f"{t1s.max():+.1f}°",
             f"[{lo1:.0f}°, {hi1:.0f}°]", ok1],
            ["θ2 codo",   f"{t2s.min():+.1f}°", f"{t2s.max():+.1f}°",
             f"[{lo2:.0f}°, {hi2:.0f}°]", ok2],
        ]
        tbl = ax_s.table(cellText=rows,
                         colLabels=["", "Min", "Max", "Rango permitido", "Estado"],
                         cellLoc="center", loc="center", bbox=[0, 0, 1, 1])
        tbl.auto_set_font_size(False); tbl.set_fontsize(9)
        for (r, c), cell in tbl.get_celld().items():
            if r == 0:
                cell.set_facecolor("#DBEAFE")
                cell.set_text_props(weight="bold", color=C_TEXT)
            elif c == 4:
                cell.set_facecolor("#FEE2E2" if "✗" in cell.get_text().get_text()
                                   else "#DCFCE7")
            cell.set_edgecolor(C_BORDER)

        self.fig2.suptitle(
            f"Analisis de rango angular · {len(data)} puntos · "
            f"limites por offset (θ1+{S1_OFFSET:.0f}°, θ2+{S2_OFFSET:.0f}°)",
            fontsize=11, fontweight="bold", color=C_TEXT, y=0.98)
        self.canvas2.draw_idle()

    # ================================================================== #
    #  DATOS DEL SENSOR (NUEVO)
    # ================================================================== #
    def _draw_sensor_chart(self):
        self.fig3.clear()
        self.fig3.set_facecolor(C_PANEL)

        if not self.scan_results:
            ax = self.fig3.add_subplot(111)
            ax.set_facecolor("#F8FAFF")
            ax.text(0.5, 0.5,
                    "Aun no hay mediciones.\n"
                    "Ejecuta 'Esquinas + centro' o 'Medir aqui' con el Arduino\n"
                    "conectado para registrar Iluminancia y Luminancia.",
                    ha="center", va="center", fontsize=11,
                    color=C_SUBTEXT, transform=ax.transAxes)
            ax.set_xticks([]); ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_edgecolor(C_BORDER)
            self.canvas3.draw_idle()
            return

        labels = [r["label"] for r in self.scan_results]
        ilum   = np.array([r["iluminancia"] for r in self.scan_results])
        lumin  = np.array([r["luminancia"]  for r in self.scan_results])
        xs     = np.array([r["x"] for r in self.scan_results])
        ys     = np.array([r["y"] for r in self.scan_results])

        gs = self.fig3.add_gridspec(2, 2, hspace=0.45, wspace=0.30,
                                    left=0.09, right=0.96,
                                    top=0.93, bottom=0.08)
        ax_bar = self.fig3.add_subplot(gs[0, :])
        ax_map = self.fig3.add_subplot(gs[1, 0])
        ax_cmp = self.fig3.add_subplot(gs[1, 1])

        # ── Barras: Iluminancia y Luminancia por punto ────────────
        x = np.arange(len(labels))
        w_bar = 0.38
        ax_bar.bar(x - w_bar/2, ilum,  w_bar, label="Iluminancia (lux)",
                   color=C_ACCENT, edgecolor="white")
        ax_bar.bar(x + w_bar/2, lumin, w_bar, label="Luminancia (cd/m²)",
                   color=C_ACCENT2, edgecolor="white")
        for i, v in enumerate(ilum):
            ax_bar.text(i - w_bar/2, v, f"{v:.1f}", ha="center",
                        va="bottom", fontsize=8, color=C_TEXT)
        for i, v in enumerate(lumin):
            ax_bar.text(i + w_bar/2, v, f"{v:.1f}", ha="center",
                        va="bottom", fontsize=8, color=C_TEXT)
        ax_bar.set_xticks(x)
        ax_bar.set_xticklabels(labels, fontsize=9)
        ax_bar.set_ylabel("Iluminancia (lux) / Luminancia (cd/m²)",
                          fontsize=9, color=C_SUBTEXT)
        ax_bar.set_title("Mediciones VEML7700 por punto",
                         fontsize=11, color=C_TEXT, pad=8)
        ax_bar.legend(fontsize=9, loc="upper right",
                      framealpha=0.9, edgecolor=C_BORDER)
        ax_bar.grid(alpha=0.15, color=C_BORDER, axis="y")

        # ── Mapa espacial: posiciones (x,y) coloreadas por Luminancia ─
        w_p, h_p = self.ancho.get(), self.alto.get()
        scr = patches.FancyBboxPatch((0, 0), w_p, h_p,
                                     boxstyle="round,pad=0.1",
                                     facecolor=C_SCREEN, alpha=0.3,
                                     edgecolor=C_ACCENT, lw=1.5)
        ax_map.add_patch(scr)
        sc = ax_map.scatter(xs, ys, c=lumin, s=220, cmap="viridis",
                            edgecolors="white", linewidths=1.5, zorder=3)
        for xi, yi, lb in zip(xs, ys, labels):
            ax_map.annotate(lb, (xi, yi), textcoords="offset points",
                            xytext=(8, 8), fontsize=8, color=C_TEXT)
        cb = self.fig3.colorbar(sc, ax=ax_map, shrink=0.85, pad=0.03)
        cb.set_label("Luminancia (cd/m²)", fontsize=9, color=C_SUBTEXT)
        cb.ax.tick_params(colors=C_SUBTEXT, labelsize=8)
        ax_map.set_xlim(-0.5, w_p + 0.5)
        ax_map.set_ylim(-0.5, h_p + 0.5)
        ax_map.set_aspect("equal")
        ax_map.set_title("Mapa espacial (Luminancia por posicion)",
                         fontsize=10, color=C_TEXT, pad=6)
        ax_map.set_xlabel("x (cm)", fontsize=9, color=C_SUBTEXT)
        ax_map.set_ylabel("y (cm)", fontsize=9, color=C_SUBTEXT)
        ax_map.grid(alpha=0.15, color=C_BORDER)

        # ── Comparativa Iluminancia vs Luminancia (dispersion) ─────
        ax_cmp.scatter(ilum, lumin, color=C_ACCENT, s=80, alpha=0.7,
                       edgecolors="white", linewidths=1.2)
        for xi, yi, lb in zip(ilum, lumin, labels):
            ax_cmp.annotate(lb, (xi, yi), textcoords="offset points",
                            xytext=(6, 6), fontsize=8, color=C_TEXT)
        ax_cmp.set_xlabel("Iluminancia (lux)", fontsize=9, color=C_SUBTEXT)
        ax_cmp.set_ylabel("Luminancia (cd/m²)", fontsize=9, color=C_SUBTEXT)
        ax_cmp.set_title(f"Iluminancia vs Luminancia (Ω={OMEGA_SR:.4f} sr)",
                         fontsize=10, color=C_TEXT, pad=6)
        ax_cmp.grid(alpha=0.15, color=C_BORDER)

        for ax in (ax_bar, ax_map, ax_cmp):
            ax.set_facecolor("#F8FAFF")
            ax.tick_params(colors=C_SUBTEXT, labelsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor(C_BORDER)

        self.fig3.suptitle(
            f"Mediciones del sensor VEML7700 · {len(self.scan_results)} puntos",
            fontsize=12, fontweight="bold", color=C_TEXT, y=0.985)
        self.canvas3.draw_idle()


# ─────────────────────────────────────────────────────────────────────────── #
if __name__ == "__main__":
    app = BrazoApp()
    app.mainloop()
