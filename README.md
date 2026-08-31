# Proyecto Final de Estudios (PFE) - Ingeniería en Mecatrónica (UNCUYO)

## Integración de Sistema de Visión Artificial y Planificación Dinámica de Trayectorias para una Celda de Clasificación Pick-and-Place

**Autor:** Matías Exequiel Molina  
**Director:** Eric Sanchez  
**Institución:** Universidad Nacional de Cuyo - Facultad de Ingeniería  
**Año:** 2026  

---

## 📁 Estructura del Repositorio

- `Firmware/`: Código fuente para STM32F446RE (C / FreeRTOS / micro-ROS). Planificación de trayectorias quínticas y cinemática analítica en la FPU.
- `ros2_ws/`: Workspace de ROS 2 Humble (Python / PyQt5 / OpenCV). Procesamiento de visión cenital, estimación determinista por compuertas espaciales y máquina de estados.
- `Informe/`: Documento formal del Proyecto Final de Estudios en LaTeX (`.tex`) y referencias bibliográficas (`.bib`).

---

## 🛠️ Especificaciones Técnicas

- **SO:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware:** ROS 2 Humble Hawksbill & micro-ROS (DDS-XRCE @ 115200 baudios)
- **Hardware:** STM32 Nucleo-F446RE, Pololu DRV8825 (1/8 micropaso), 3x Encoders AS5600 (I2C 400 kHz), Multiplexor TCA9548A, Cámara Cenital HD 720p @ 30 FPS.
