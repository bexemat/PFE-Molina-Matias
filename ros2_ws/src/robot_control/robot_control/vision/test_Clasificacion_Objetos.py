#!/usr/bin/env python3
import os
import cv2
import numpy as np

SYMLINK_PATH = '/dev/v4l/by-id/usb-Generic_HD_camera-video-index0'

def main():
    if os.path.exists(SYMLINK_PATH):
        camera_path = os.path.realpath(SYMLINK_PATH)
        print(f"🔗 Symlink encontrado. Apuntando a: {camera_path}")
    else:
        print("⚠️ No se encontró el symlink fijo. Usando /dev/video10 o índice 0.")
        camera_path = '/dev/video10'

    cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Error: No se pudo acceder a la webcam.")
        return

    # Configuración de cámara
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Valores HSV optimizados
    lower_hsv = np.array([136, 64, 219])
    upper_hsv = np.array([179, 255, 255])

    # Kernels morfológicos
    kernel_close = np.ones((11, 11), np.uint8)
    kernel_open = np.ones((5, 5), np.uint8)

    print("\n" + "=" * 50)
    print("👁️  SCRIPT DE PRUEBA DE CLASIFICACIÓN (EXTENT + RECT RATIO)")
    print("Presiona 'q' o 'ESC' para salir.")
    print("=" * 50 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Filtros morfológicos
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            for cnt in contours:
                area_objeto = cv2.contourArea(cnt)
                if area_objeto > 80:
                    
                    # 1. Caja recta (Upright Bounding Box)
                    rect_upright = cv2.boundingRect(cnt)
                    area_upright = rect_upright[2] * rect_upright[3]
                    
                    # 2. Caja rotada (Min Area Rect)
                    rect_rot = cv2.minAreaRect(cnt)
                    w_rot, h_rot = rect_rot[1]
                    
                    if w_rot == 0 or h_rot == 0 or area_upright == 0:
                        continue
                        
                    area_rot = w_rot * h_rot
                    
                    # 3. Métricas Geométricas
                    extent_rotado = float(area_objeto) / area_rot
                    ratio_cajas = area_rot / area_upright
                    
                    # 4. Lógica de Clasificación Robusta
                    # Si ratio_cajas < 0.85 -> El objeto está rotado (Como el cono no rueda, es un Cubo)
                    # Si ratio_cajas >= 0.85 -> Está derecho, evaluamos su densidad (Extent > 0.85 es Cubo)
                    if ratio_cajas < 0.85 or extent_rotado > 0.85:
                        forma = "Cubo"
                        color_texto = (0, 255, 255) # Amarillo
                    else:
                        forma = "Cono"
                        color_texto = (255, 100, 255) # Rosa/Lila
                    
                    # --- DIBUJO ---
                    box = cv2.boxPoints(rect_rot)
                    box = np.intp(box)
                    cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
                    
                    x_centro = int(rect_rot[0][0])
                    y_centro = int(rect_rot[0][1])
                    
                    texto = f"{forma} (E:{extent_rotado:.2f} R:{ratio_cajas:.2f})"
                    cv2.putText(frame, texto, (x_centro - 60, y_centro - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_texto, 2)

        cv2.imshow("Prueba de Clasificacion (Rotada)", frame)
        cv2.imshow("Mascara (Segmentacion)", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()