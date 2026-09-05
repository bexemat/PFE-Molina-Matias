#!/usr/bin/env python3
"""Herramienta interactiva con barras de control para calibración de umbrales HSV.

Permite sintonizar dinámicamente los rangos de segmentación cromática bajo variaciones
de luz ambiental, visualizar la máscara binaria resultante tras el filtrado morfológico
(cierre y apertura) y verificar la relación de aspecto (Aspect Ratio) de los objetos.
"""

import os
import cv2
import numpy as np

# Ruta persistente udev asignada al sensor de captura
SYMLINK_PATH: str = "/dev/v4l/by-id/usb-Generic_HD_camera-video-index0"


def on_trackbar_change(value: int) -> None:
    """Callback nulo requerido por las interfaces de control deslizante en OpenCV.

    Args:
        value (int): Posición actual de la barra de desplazamiento.
    """
    pass


def main() -> None:
    """Inicializa la captura de video y administra el lazo de calibración cromática."""
    if os.path.exists(SYMLINK_PATH):
        camera_path = os.path.realpath(SYMLINK_PATH)
    else:
        camera_path = "/dev/video10"

    cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se pudo establecer comunicación con el dispositivo de video.")
        return

    # Instanciación de ventanas gráficas para ajuste y monitoreo
    cv2.namedWindow("Ajuste HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mascara HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Camara Principal", cv2.WINDOW_NORMAL)

    # Controles deslizantes para sintonización de planos de Hue, Saturation y Value
    cv2.createTrackbar("H Min", "Ajuste HSV", 140, 179, on_trackbar_change)
    cv2.createTrackbar("H Max", "Ajuste HSV", 175, 179, on_trackbar_change)
    cv2.createTrackbar("S Min", "Ajuste HSV", 50, 255, on_trackbar_change)
    cv2.createTrackbar("S Max", "Ajuste HSV", 255, 255, on_trackbar_change)
    cv2.createTrackbar("V Min", "Ajuste HSV", 50, 255, on_trackbar_change)
    cv2.createTrackbar("V Max", "Ajuste HSV", 255, 255, on_trackbar_change)

    # Elementos estructurantes para mitigación de ruido
    kernel_close = np.ones((11, 11), dtype=np.uint8)  # Relleno de discontinuidades
    kernel_open = np.ones((5, 5), dtype=np.uint8)     # Atenuación de ruido en fondo

    print("\n" + "=" * 50)
    print("INTERFAZ DE CALIBRACIÓN DE ESPACIO DE COLOR HSV")
    print("1. Ajuste los deslizadores hasta aislar la silueta del objeto en la máscara.")
    print("2. Presione 'q' o ESC para cerrar y volcar la configuración al estándar de producción.")
    print("=" * 50 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        h_min = cv2.getTrackbarPos("H Min", "Ajuste HSV")
        h_max = cv2.getTrackbarPos("H Max", "Ajuste HSV")
        s_min = cv2.getTrackbarPos("S Min", "Ajuste HSV")
        s_max = cv2.getTrackbarPos("S Max", "Ajuste HSV")
        v_min = cv2.getTrackbarPos("V Min", "Ajuste HSV")
        v_max = cv2.getTrackbarPos("V Max", "Ajuste HSV")

        lower_hsv = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_hsv = np.array([h_max, s_max, v_max], dtype=np.uint8)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Filtros morfológicos secuenciales
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 80:
                moments = cv2.moments(largest)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    aspect_ratio = float(w) / h if h != 0 else 0.0

                    cv2.putText(
                        frame,
                        f"Area: {int(area)}px",
                        (cx + 10, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"AR: {aspect_ratio:.2f}",
                        (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )

        cv2.putText(
            frame,
            f"Lower: [{h_min}, {s_min}, {v_min}]",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"Upper: [{h_max}, {s_max}, {v_max}]",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
        )

        cv2.imshow("Camara Principal", frame)
        cv2.imshow("Mascara HSV", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            print("\nBloque de configuración para vision_detector_node.py:\n")
            print(f"self.lower_hsv = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"self.upper_hsv = np.array([{h_max}, {s_max}, {v_max}])\n")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()