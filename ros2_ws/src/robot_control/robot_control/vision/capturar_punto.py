#!/usr/bin/env python3
import cv2
import numpy as np

CAMERA_PATH = '/dev/v4l/by-id/usb-Generic_HD_camera-video-index0'

# Umbrales HSV para el cubo rosa
LOWER_HSV = np.array([161, 91, 128])
UPPER_HSV = np.array([179, 255, 255])

# Abrir cámara y descartar los primeros fotogramas para estabilizar brillo
cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
for _ in range(10):
    ret, frame = cap.read()

cap.release()

if not ret:
    print("❌ Error al capturar imagen de la webcam.")
    exit()

# Procesamiento de imagen
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
kernel = np.ones((5, 5), np.uint8)
mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) > 150:
        M = cv2.moments(largest)
        if M["m00"] != 0:
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
            print(f"\n✅ CAPTURA EXITOSA -> X_px = {cx:.2f} | Y_px = {cy:.2f}\n")
        else:
            print("⚠️ No se pudo calcular el centroide.")
    else:
        print("⚠️ Objeto demasiado pequeño.")
else:
    print("❌ No se detectó el cubo rosa en la escena.")