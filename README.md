# Proyecto Final de Estudios (PFE) - Ingeniería en Mecatrónica (UNCUYO)

### Integración de Sistema de Visión Artificial y Planificación Dinámica de Trayectorias para una Celda de Clasificación Pick-and-Place

* **Autor:** Matías Exequiel Molina
* **Director:** Eric Sanchez
* **Institución:** Universidad Nacional de Cuyo - Facultad de Ingeniería
* **Año:** 2026

---

## 📌 Resumen del Proyecto

Este trabajo constituye una evolución técnica y funcional del manipulador paralelo de 3 GDL desarrollado previamente por Agüero y Lezcano. Transforma una maqueta experimental estática en una celda de manufactura orientada a estándares industriales, capaz de interceptar y clasificar piezas en movimiento continuo sobre una cinta transportadora mediante visión artificial en tiempo real y arquitectura de software distribuida.

### Puntos Clave de la Reingeniería
* **Control Embebido Determinista (Firmware C / FreeRTOS):** Migración de la cinemática inversa analítica y del planificador de trayectorias quínticas ($C^2$) hacia la FPU del microcontrolador STM32F446RE, desacoplando el lazo de control del tráfico serial en ROS 2. Lazo de movimiento determinista a 100 Hz (`mControlTask`) con PWM generado por hardware (TIM2, TIM3, TIM13) y lectura de encoders magnéticos absolutos AS5600 (12 bits) vía I2C a 400 kHz con multiplexor TCA9548A.
* **Percepción Visual en Tiempo Real (ROS 2 / OpenCV):** Estimación determinista de la velocidad lineal de avance mediante compuertas espaciales fijas ($\Delta Y = 70.0\text{ mm}$) a 30 FPS. Clasificación geométrica de piezas (Cubo vs. Cono) invariante a la rotación mediante el análisis combinado de *Extent* y relación de envolventes (*Rect Ratio*).
* **Supervisión y Orquestación:** Máquina de estados en ROS 2 con compensación temporal de magnetización ($T_{anticipo} = 0.15\text{ s}$), verificación en lazo cerrado de posición de reposo ($\pm2.0\text{ mm}$), despegue vertical puro ($Z_{lift} = 125.0\text{ mm}$) para suprimir fuerzas cortantes y retorno rápido evasivo ante fallos de retención.
* **Mecánica y Hardware:** Unión por chavetero mecanizada en los ejes de los motores paso a paso y engranajes impresos en 3D para neutralizar holguras y deslizamiento torsional. Acondicionamiento de pares trenzados blindados en el bus I2C (SCL/GND y SDA/VCC) para suprimir interferencias electromagnéticas. Integración de una cinta transportadora a escala diseñada en CAD (SolidWorks).

---

## 🛠️ Especificaciones Técnicas

* **Sistema Operativo:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware Robótico:** ROS 2 Humble Hawksbill & micro-ROS (XRCE-DDS serial @ 115200 baudios)
* **Herramientas de Firmware:** STM32CubeIDE v1.14+, FreeRTOS v10.3, ARM GCC Toolchain
* **Hardware de Control:** STM32 Nucleo-F446RE, 3x Controladores Pololu DRV8825 (configurados a 1/8 de micropaso)
* **Sensores:** 3x Encoders magnéticos absolutos de 12 bits AS5600 (I2C @ 400 kHz), Multiplexor TCA9548A
* **Visión:** Cámara Cenital HD 720p @ 30 FPS conectada mediante symlink persistente V4L2

---

## 📁 Estructura del Repositorio

```text
robot_workspace/
├── Firmware/                      # Proyecto STM32CubeIDE (C / FreeRTOS / micro-ROS)
│   ├── Core/
│   │   ├── Inc/                   # kinematics.h, trajectory_planner.h, motors.h, main.h, etc.
│   │   └── Src/                   # kinematics.c, trajectory_planner.c, tasks.c, main.c, etc.
│   └── micro_ros_stm32cubemx_utils/
├── ros2_ws/                       # Workspace de ROS 2 Humble
│   └── src/
│       ├── extra_interfaces/      # Mensajes personalizados ROS 2 (Trama.msg)
│       ├── robot_bringup/         # Archivos de lanzamiento consolidado (robot.launch.py)
│       └── robot_control/         # Paquete de control, visión y GUI
│           ├── gui/               # Interfaz V1 (robot_ui_node) y widgets Matplotlib
│           ├── gui_pickAndPlace_Final/ # Interfaz V2 continua (robot_ui_node_2) y FSM
│           ├── orchestrator/      # Nodo puente orquestador (publisher.py)
│           ├── planning/          # Herramientas auxiliares de planificación
│           └── vision/            # Nodo de visión OpenCV y scripts de calibración
├── Informe/                       # Documento formal del PFE en LaTeX (.tex) y bibliografía (.bib)
└── README.md
