import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import threading
import time
import math

# Configuración del tema oscuro
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class PanelControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Panel de Control - Arduino PID")
        self.geometry("1100x650")
        self.configure(fg_color="#1A1A1A") # Fondo oscuro

        # Variables de control
        self.arduino = None
        self.leyendo = False
        self.datos_x = []
        self.datos_y_pv = [] # Proceso (Temperatura)
        self.datos_y_sp = [] # Setpoint
        self.inicio_tiempo = time.time()

        # ==================== PANEL IZQUIERDO ====================
        self.frame_izq = ctk.CTkFrame(self, width=320, fg_color="transparent")
        self.frame_izq.pack(side="left", fill="y", padx=20, pady=20)

        self.lbl_titulo = ctk.CTkLabel(self.frame_izq, text="PANEL DE CONTROL", font=("BankGothic", 22, "bold"), text_color="white")
        self.lbl_titulo.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # --- Fila 1: Botón Start y Switch Manual/Digital ---
        self.btn_start = ctk.CTkButton(self.frame_izq, text="Start", fg_color="#28A745", hover_color="#218838", font=("Arial", 16, "bold"), height=50, command=self.enviar_start)
        self.btn_start.grid(row=1, column=0, sticky="ew", padx=(0,10), pady=5)

        self.switch_modo = ctk.CTkSwitch(self.frame_izq, text="Manual", font=("Arial", 12, "bold"), command=self.cambiar_modo)
        self.switch_modo.grid(row=1, column=1, sticky="w")

        # --- Fila 2: Botón Stop y Etiqueta del Switch ---
        self.btn_stop = ctk.CTkButton(self.frame_izq, text="Stop", fg_color="#DC3545", hover_color="#C82333", font=("Arial", 16, "bold"), height=50, command=self.enviar_stop)
        self.btn_stop.grid(row=2, column=0, sticky="ew", padx=(0,10), pady=5)

        self.lbl_modo = ctk.CTkLabel(self.frame_izq, text="Digital / Manual", font=("Arial", 12, "bold"))
        self.lbl_modo.grid(row=2, column=1, sticky="w")

        # --- Fila 3: Puerto y Botón Conectar ---
        self.lbl_puerto = ctk.CTkLabel(self.frame_izq, text="Puerto", font=("Arial", 14))
        self.lbl_puerto.grid(row=3, column=0, pady=20)

        puertos = [p.device for p in serial.tools.list_ports.comports()]
        if not puertos: puertos = ["COM5"]
        self.cmb_puertos = ctk.CTkComboBox(self.frame_izq, values=puertos, height=35)
        self.cmb_puertos.set(puertos[0])
        self.cmb_puertos.grid(row=3, column=1, sticky="ew", pady=20)

        self.btn_conectar = ctk.CTkButton(self.frame_izq, text="Conectar Serial", fg_color="#007ACC", font=("Arial", 14, "bold"), command=self.alternar_conexion)
        self.btn_conectar.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(0, 20))

        # --- Filas 4-7: Campos de Parámetros (SP, Kp, Ki, Kd) ---
        self.txt_sp = self.crear_parametro(5, "Temperatura (SP)", "50.0", "white", "black")
        self.txt_kp = self.crear_parametro(6, "KpEF", "2.5", "#7F7F7F", "white")
        self.txt_ki = self.crear_parametro(7, "KiEF", "0.1", "#7F7F7F", "white")
        self.txt_kd = self.crear_parametro(8, "KdEF", "0.8", "#7F7F7F", "white")

        # Vincular la tecla "Enter" para enviar valores al Arduino
        self.txt_sp.bind("<Return>", lambda e: self.enviar_valor('S', self.txt_sp.get()))
        self.txt_kp.bind("<Return>", lambda e: self.enviar_valor('P', self.txt_kp.get()))
        self.txt_ki.bind("<Return>", lambda e: self.enviar_valor('I', self.txt_ki.get()))
        self.txt_kd.bind("<Return>", lambda e: self.enviar_valor('D', self.txt_kd.get()))

        # ==================== PANEL DERECHO ====================
        self.frame_der = ctk.CTkFrame(self, fg_color="transparent")
        self.frame_der.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        # --- Gráfica Superior ---
        self.fig, self.ax = plt.subplots(figsize=(6, 3), facecolor="#111111")
        self.ax.set_facecolor("#111111")
        self.ax.set_title("Temp-Time", color="white")
        self.ax.set_xlabel("X", color="white")
        self.ax.set_ylabel("Y", color="white")
        self.ax.tick_params(colors="white")
        self.ax.set_ylim(0, 150)
        self.ax.set_xlim(0, 60)
        
        # Dos líneas: Temp(Roja) y SP(Azul)
        self.linea_pv, = self.ax.plot([], [], 'r.', markersize=6, label="Temp")
        self.linea_sp, = self.ax.plot([], [], 'b.', markersize=6, label="SP")
        self.fig.tight_layout()

        self.canvas_grafica = FigureCanvasTkAgg(self.fig, master=self.frame_der)
        self.canvas_grafica.get_tk_widget().pack(fill="both", expand=True)

        # --- Medidores Inferiores ---
        self.frame_medidores = ctk.CTkFrame(self.frame_der, fg_color="transparent", height=250)
        self.frame_medidores.pack(fill="x", side="bottom", pady=10)

        # Medidor Circular (Actuador)
        self.frame_act = ctk.CTkFrame(self.frame_medidores, fg_color="transparent")
        self.frame_act.pack(side="left", expand=True)
        
        self.canvas_actuador = ctk.CTkCanvas(self.frame_act, width=200, height=180, bg="#1A1A1A", highlightthickness=0)
        self.canvas_actuador.pack()
        self.dibujar_fondo_actuador()
        self.aguja_act = self.canvas_actuador.create_line(100, 100, 43, 156, fill="red", width=4) # Aguja inicial
        
        ctk.CTkLabel(self.frame_act, text="Actuador", font=("Arial", 16)).pack()

        # Medidor Lineal (Termómetro)
        self.frame_temp = ctk.CTkFrame(self.frame_medidores, fg_color="transparent")
        self.frame_temp.pack(side="right", expand=True)

        self.bar_temp = ctk.CTkProgressBar(self.frame_temp, orientation="vertical", width=25, height=150, fg_color="#333333", progress_color="red")
        self.bar_temp.set(0)
        self.bar_temp.pack(pady=15)
        
        self.lbl_temp_val = ctk.CTkLabel(self.frame_temp, text="0 °C", font=("Arial", 14, "bold"))
        self.lbl_temp_val.place(relx=0.5, rely=0.5, anchor="center") # Texto centrado en la barra
        
        ctk.CTkLabel(self.frame_temp, text="Temperatura actual", font=("Arial", 16)).pack()

    def crear_parametro(self, fila, nombre, val_defecto, bg_color, text_color):
        lbl = ctk.CTkLabel(self.frame_izq, text=nombre, font=("Arial", 12, "bold"), fg_color=bg_color, text_color=text_color, width=140, height=35)
        lbl.grid(row=fila, column=0, pady=5, padx=5, sticky="e")
        
        txt = ctk.CTkEntry(self.frame_izq, height=35, justify="center", font=("Arial", 14, "bold"))
        txt.insert(0, val_defecto)
        txt.grid(row=fila, column=1, pady=5, sticky="ew")
        return txt

    def dibujar_fondo_actuador(self):
        # Dibuja el arco del velocímetro
        self.canvas_actuador.create_arc(20, 20, 180, 180, start=-45, extent=270, style="arc", outline="white", width=4)
        for i in range(0, 101, 10):
            angulo = math.radians(225 - (i / 100.0) * 270)
            x_text = 100 + 60 * math.cos(angulo)
            y_text = 100 - 60 * math.sin(angulo)
            self.canvas_actuador.create_text(x_text, y_text, text=str(i), fill="white", font=("Arial", 10))

    def actualizar_medidores(self, temp, sp, actuador):
        # 1. Actualizar Termómetro Lineal (Max 150)
        self.bar_temp.set(min(temp / 150.0, 1.0))
        self.lbl_temp_val.configure(text=f"{temp:.1f}")

        # 2. Actualizar Aguja del Actuador Circular (0 a 100%)
        actuador = max(0, min(actuador, 100)) # Limitar entre 0 y 100
        angulo = math.radians(225 - (actuador / 100.0) * 270)
        x_fin = 100 + 70 * math.cos(angulo)
        y_fin = 100 - 70 * math.sin(angulo)
        self.canvas_actuador.coords(self.aguja_act, 100, 100, x_fin, y_fin)

        # 3. Actualizar Gráfica con Autoajuste
        tiempo_actual = time.time() - self.inicio_tiempo
        self.datos_x.append(tiempo_actual)
        self.datos_y_pv.append(temp)
        self.datos_y_sp.append(sp)

        # --- AUTOAJUSTE EN EL EJE X (Tiempo: ventana móvil de 60 segundos) ---
        if tiempo_actual > 60:
            self.ax.set_xlim(tiempo_actual - 60, tiempo_actual)
            # Limpiar memoria si el arreglo crece demasiado
            if len(self.datos_x) > 1000:
                self.datos_x = self.datos_x[-500:]
                self.datos_y_pv = self.datos_y_pv[-500:]
                self.datos_y_sp = self.datos_y_sp[-500:]
        else:
            self.ax.set_xlim(0, 60)

        # --- AUTOAJUSTE EN EL EJE Y (Temperatura Dinámica) ---
        todos_los_valores = self.datos_y_pv + self.datos_y_sp
        if todos_los_valores:
            min_y = min(todos_los_valores)
            max_y = max(todos_los_valores)
            
            margen = 5
            lim_inferior = max(0, min_y - margen)
            lim_superior = max_y + margen
            
            if lim_superior - lim_inferior < 10:
                lim_superior = lim_inferior + 10
                
            self.ax.set_ylim(lim_inferior, lim_superior)

        # Dibujar las líneas modificadas
        self.linea_pv.set_data(self.datos_x, self.datos_y_pv)
        self.linea_sp.set_data(self.datos_x, self.datos_y_sp)
        self.canvas_grafica.draw_idle()

    # ================= FUNCIONES DE COMUNICACIÓN CON ARDUINO =================
    def alternar_conexion(self):
        if not self.leyendo:
            puerto = self.cmb_puertos.get()
            try:
                self.arduino = serial.Serial(puerto, 115200, timeout=1)
                time.sleep(1.5) # Esperar a que el Arduino reinicie
                self.leyendo = True
                threading.Thread(target=self.leer_serial, daemon=True).start()
                
                self.btn_conectar.configure(text="Desconectar", fg_color="#DC3545")
                # Enviar valores iniciales al conectar
                self.enviar_valor('S', self.txt_sp.get())
                self.enviar_valor('P', self.txt_kp.get())
            except Exception as e:
                print(f"Error: {e}")
        else:
            self.leyendo = False
            if self.arduino: self.arduino.close()
            self.btn_conectar.configure(text="Conectar Serial", fg_color="#007ACC")

    def leer_serial(self):
        while self.leyendo:
            if self.arduino.in_waiting > 0:
                try:
                    linea = self.arduino.readline().decode('utf-8').strip()
                    datos = linea.split(',')
                    if len(datos) >= 3:
                        temp = float(datos[0])
                        sp = float(datos[1])
                        act = float(datos[2])
                        # Programar actualización en el hilo principal
                        self.after(0, self.actualizar_medidores, temp, sp, act)
                except Exception:
                    pass
            time.sleep(0.05)

    def enviar_start(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.write(b"A1\n")

    def enviar_stop(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.write(b"A0\n")

    def cambiar_modo(self):
        if self.arduino and self.arduino.is_open:
            val = 1 if self.switch_modo.get() == 1 else 0
            self.arduino.write(f"M{val}\n".encode())

    def enviar_valor(self, prefijo, valor_str):
        try:
            val = float(valor_str)
            if self.arduino and self.arduino.is_open:
                comando = f"{prefijo}{val}\n"
                self.arduino.write(comando.encode())
                print(f"Enviado al Arduino: {comando.strip()}")
        except ValueError:
            print("Por favor, ingresa un número válido.")

if __name__ == "__main__":
    app = PanelControlApp()
    app.mainloop()