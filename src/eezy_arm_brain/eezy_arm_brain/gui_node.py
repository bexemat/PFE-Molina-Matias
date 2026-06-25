#!/usr/bin/env python3
import sys
import threading
import customtkinter as ctk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configuración estética global de la interfaz
ctk.set_appearance_mode("System")  # Detecta si tu Ubuntu está en modo claro u oscuro
ctk.set_default_color_theme("blue") # Tema de color principal para botones y barras

class ArmGuiNode(Node):
    """Nodo de ROS 2 encargado de la comunicación (Tópicos)"""
    def __init__(self, update_gui_callback):
        super().__init__('arm_gui_node')
        self.update_gui_callback = update_gui_callback
        
        # Publicador de consignas
        self.publisher_ = self.create_publisher(String, 'arm_commands', 10)
        
        # Suscriptor de telemetría real (desde la STM32)
        self.subscription = self.create_subscription(
            String, 'arm_states', self.listener_callback, 10)
        
        self.get_logger().info('Lazo de comunicación ROS 2 - GUI inicializado.')

    def enviar_consigna(self, msg_texto):
        msg = String()
        msg.data = msg_texto
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        # Envía los datos recibidos al hilo de la interfaz gráfica
        self.update_gui_callback(msg.data)


class ArmGuiWindow(ctk.CTk):
    """Ventana Principal de la Interfaz Gráfica (Moderna)"""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        # Configuración de ventana
        self.title("EEZYbotARM - Panel de Control Pro")
        self.geometry("500x600")
        self.resizable(False, False)
        
        # --- CONTENEDOR DE CONSIGNAS (EMISOR) ---
        self.frame_cmd = ctk.CTkFrame(self, corner_radius=15)
        self.frame_cmd.pack(fill="both", expand=True, padx=20, pady=15)
        
        ctk.CTkLabel(self.frame_cmd, text="CONSIGNAS DE ARTICULACIÓN", font=("Helvetica", 16, "bold")).pack(pady=15)
        
        # Sliders modernos con lectura digital en tiempo real
        self.val_base = ctk.StringVar(value="90°")
        self.slider_base = self.crear_control_articulacion(self.frame_cmd, "Base (q1):", 0, 180, self.val_base)
        
        self.val_hombro = ctk.StringVar(value="90°")
        self.slider_hombro = self.crear_control_articulacion(self.frame_cmd, "Hombro (q2):", 0, 180, self.val_hombro)
        
        self.val_codo = ctk.StringVar(value="90°")
        self.slider_codo = self.crear_control_articulacion(self.frame_cmd, "Codo (q3):", 0, 180, self.val_codo)
        
        # Switch moderno para el Solenoide
        self.switch_solenoide = ctk.CTkSwitch(self.frame_cmd, text="Activar Efector Final (Solenoide)", 
                                             font=("Helvetica", 13), command=self.publicar_comandos)
        self.switch_solenoide.pack(pady=20)
        
        # --- CONTENEDOR DE TELEMETRÍA (RECEPTOR) ---
        self.frame_status = ctk.CTkFrame(self, corner_radius=15, fg_color=("#EAEAEA", "#252525"))
        self.frame_status.pack(fill="both", expand=False, padx=20, pady=15)
        
        ctk.CTkLabel(self.frame_status, text="TELEMETRÍA EN REAL DESDE STM32", font=("Helvetica", 14, "bold"), text_color="#1F6AA5").pack(pady=10)
        
        self.lbl_real = ctk.CTkLabel(self.frame_status, text="Esperando conexión con microcontrolador...", font=("Courier New", 13), text_color=("#333333", "#A0A0A0"))
        self.lbl_real.pack(pady=15)

    def crear_control_articulacion(self, master, label_text, min_v, max_v, string_var):
        """Genera una fila estética con etiqueta, slider y valor numérico dinámico"""
        frame_row = ctk.CTkFrame(master, fg_color="transparent")
        frame_row.pack(fill="x", padx=25, pady=8)
        
        ctk.CTkLabel(frame_row, text=label_text, width=100, anchor="w", font=("Helvetica", 13)).pack(side="left")
        
        slider = ctk.CTkSlider(frame_row, from_=min_v, to=max_v, number_of_steps=180, command=lambda v: self.on_slider_move(v, string_var))
        slider.set(90)
        slider.pack(side="left", fill="x", expand=True, padx=10)
        
        ctk.CTkLabel(frame_row, textvariable=string_var, width=50, font=("Helvetica", 13, "bold")).pack(side="right")
        return slider

    def on_slider_move(self, valor, string_var):
        # Actualiza la etiqueta de texto flotante (° de la articulación)
        string_var.set(f"{int(valor)}°")
        self.publicar_comandos()

    def publicar_comandos(self):
        # Captura los datos de la GUI y los formatea
        q1 = int(self.slider_base.get())
        q2 = int(self.slider_hombro.get())
        q3 = int(self.slider_codo.get())
        solenide_on = 1 if self.switch_solenoide.get() else 0
        
        # Generamos la cadena limpia que viajará por ROS
        cmd_string = f"CMD_ANGULOS: q1={q1}, q2={q2}, q3={q3}, SOL={solenide_on}"
        self.ros_node.enviar_consigna(cmd_string)

    def actualizar_datos_reales(self, string_datos):
        # Actualiza de forma segura la telemetría en el hilo de la interfaz
        self.lbl_real.configure(text=string_datos)


def main(args=None):
    rclpy.init(args=args)
    
    def ros_to_gui_bridge(datos):
        window.actualizar_datos_reales(datos)

    # Inicializamos el Nodo de ROS 2
    ros_node = ArmGuiNode(update_gui_callback=ros_to_gui_bridge)
    
    # Hilo secundario para que ROS 2 no bloquee la interfaz gráfica
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()
    
    # Hilo principal: Corre la ventana moderna
    window = ArmGuiWindow(ros_node)
    window.mainloop()
    
    # Cierre limpio del sistema
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()