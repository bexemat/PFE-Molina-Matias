#!/usr/bin/env python3
"""Captura instantánea de un único fotograma para registrar coordenadas del centroide."""

import cv2
import numpy as np

CAMERA_PATH: str = "/dev/v4l/by-id/usb-Generic_HD_camera-video-index0"
LOWER_HSV: np.ndarray = np.array([161, 91, 128], dtype=np.uint8)
UPPER_HSV: np.ndarray = np.array([179, 255, 255], dtype=np.uint8)


def main() -> None:
    cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    # Descarte inicial para estabilizar auto-exposición del sensor
    for _ in range(10):
        ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Error: No se pudo capturar imagen de la cámara.")
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 150:
            moments = cv2.moments(largest)
            if moments["m00"] != 0:
                cx = float(moments["m10"] / moments["m00"])
                cy = float(moments["m01"] / moments["m00"])
                print(f"Centroide capturado: X_px = {cx:.2f} | Y_px = {cy:.2f}")
            else:
                print("Error: Momento de orden cero nulo.")
        else:
            print("Advertencia: Contorno inferior al umbral mínimo de área.")
    else:
        print("Error: No se detectaron objetos en la ROI.")


if __name__ == "__main__":
    main()