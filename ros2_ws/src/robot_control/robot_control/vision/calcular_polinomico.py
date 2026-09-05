#!/usr/bin/env python3
"""Rutina interactiva de calibración espacial para mapeo cámara-robot.

Calcula un modelo de regresión cuadrática (Y_robot = a * X_px^2 + b * X_px + c)
a partir de pares empíricos de coordenadas de píxel (centroide óptico) y coordenadas
métricas reales provistas por la cinemática directa (FK). Este ajuste mitiga la
distorsión periférica y aberraciones geométricas ("ojo de pez") de la lente.
"""

from typing import List, Tuple
import numpy as np


def compute_polynomial_calibration(
    x_pixels: List[float], y_robot: List[float]
) -> Tuple[float, float, float]:
    """Ajusta un polinomio de segundo orden por el método de mínimos cuadrados.

    Args:
        x_pixels (List[float]): Muestras centroidales en el eje horizontal de la imagen [px].
        y_robot (List[float]): Posiciones longitudinales correspondientes del efector [mm].

    Returns:
        Tuple[float, float, float]: Coeficientes (a, b, c) del polinomio cuadrático:
            Y_robot = a * X_px^2 + b * X_px + c
    """
    coeffs = np.polyfit(x_pixels, y_robot, 2)
    return float(coeffs[0]), float(coeffs[1]), float(coeffs[2])


def main() -> None:
    """Punto de entrada de la rutina de calibración interactiva por consola."""
    print("\n================ CALCULADORA POLINÓMICA 2DO GRADO ================\n")

    x_pixels: List[float] = []
    y_robot: List[float] = []

    while True:
        try:
            n_raw = input("Ingrese cantidad de puntos a medir (recomendado 6 a 8): ")
            n = int(n_raw)
            if n < 3:
                print("Error: Se requieren al menos 3 puntos para un ajuste cuadrático.")
                continue
            break
        except ValueError:
            print("Entrada inválida. Ingrese un número entero.")

    for i in range(n):
        while True:
            try:
                px = float(input(f"Punto P{i+1} -> Ingrese X_px obtenido [px]: "))
                yr = float(input(f"Punto P{i+1} -> Ingrese Y del robot [FK] [mm]: "))
                x_pixels.append(px)
                y_robot.append(yr)
                break
            except ValueError:
                print("Valor inválido. Ingrese números decimales válidos.")

    a, b, c = compute_polynomial_calibration(x_pixels, y_robot)

    print("\n================ COEFICIENTES OBTENIDOS ================")
    print(f"a = {a:.10f}")
    print(f"b = {b:.10f}")
    print(f"c = {c:.10f}")
    print(f"\nFórmula resultante: Y_robot = ({a:.8f})*X_px^2 + ({b:.8f})*X_px + ({c:.8f})")
    print("========================================================\n")


if __name__ == "__main__":
    main()
