#!/usr/bin/env python3
import os
import cv2
import numpy as np

SYMLINK_PATH = '/dev/v4l/by-id/usb-Generic_HD_camera-video-index0'

def nothing(x):
    pass

def main():
    if os.path.exists(SYMLINK_PATH):
        camera_path = os.path.realpath(SYMLINK_PATH)
    else:
        camera_path = '/dev/video10'

    cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: No se pudo acceder a la webcam.")
        return

    # --- OPCIONAL: Fijar exposición manual para evitar que cambie el color ---
    # Según el driver de Linux, los comandos varían. Podés descomentar y probar:
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # 1 = Manual, 3 = Auto (a veces 0.25)
    # cap.set(cv2.CAP_PROP_EXPOSURE, 100)    # Ajustar a prueba y error
    # -------------------------------------------------------------------------

    # Crear ventanas para la interfaz de calibración
    cv2.namedWindow("Ajuste HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mascara HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Camara Principal", cv2.WINDOW_NORMAL)

    # Trackbars con rangos sugeridos
    cv2.createTrackbar("H Min", "Ajuste HSV", 140, 179, nothing)
    cv2.createTrackbar("H Max", "Ajuste HSV", 175, 179, nothing)
    cv2.createTrackbar("S Min", "Ajuste HSV", 50, 255, nothing)
    cv2.createTrackbar("S Max", "Ajuste HSV", 255, 255, nothing)
    cv2.createTrackbar("V Min", "Ajuste HSV", 50, 255, nothing)
    cv2.createTrackbar("V Max", "Ajuste HSV", 255, 255, nothing)

    # --- MEJORA 1: Kernels de distinto tamaño ---
    kernel_close = np.ones((11, 11), np.uint8) # Kernel grande para rellenar huecos
    kernel_open = np.ones((5, 5), np.uint8)    # Kernel chico para limpiar ruido del fondo

    print("\n" + "=" * 50)
    print("🛠️ SCRIPT DE CALIBRACIÓN HSV MEJORADO")
    print("1. Ajusta los sliders en la ventana 'Ajuste HSV'.")
    print("2. Asegúrate de que el objeto se vea como un bloque sólido en 'Mascara HSV'.")
    print("3. Anota los valores de 'AR' (Aspect Ratio) del cubo y del cono.")
    print("4. Presiona 'q' o 'ESC' para finalizar.")
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

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # --- MEJORA 2: Inversión de filtros morfológicos ---
        # 1. Cierre fuerte para unir las manchas separadas del objeto
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        # 2. Apertura suave para limpiar puntitos que hayan quedado en el fondo negro
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 80:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # --- MEJORA 3: Dibujo del Bounding Box y cálculo de Aspect Ratio ---
                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    aspect_ratio = float(w) / h if h != 0 else 0
                    
                    cv2.putText(frame, f"Area: {int(area)}px", (cx + 10, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"AR: {aspect_ratio:.2f}", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.putText(frame, f"Lower: [{h_min}, {s_min}, {v_min}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Upper: [{h_max}, {s_max}, {v_max}]", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Camara Principal", frame)
        cv2.imshow("Mascara HSV", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("\n📋 CÓDIGO LISTO PARA PEGAR EN vision_detector_node.py:\n")
            print(f"self.lower_hsv = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"self.upper_hsv = np.array([{h_max}, {s_max}, {v_max}])\n")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()