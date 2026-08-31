#!/usr/bin/env python3
import numpy as np

print("\n================ CALCULADORA POLINÓMICA 2DO GRADO ================\n")

x_pixels = []
y_robot = []

n = int(input("Ingrese cantidad de puntos a medir (recomendado 6 a 8): "))

for i in range(n):
    px = float(input(f"Punto P{i+1} -> Ingrese X_px obtenido: "))
    yr = float(input(f"Punto P{i+1} -> Ingrese Y del robot [FK] (mm): "))
    x_pixels.append(px)
    y_robot.append(yr)

# Ajuste Cuadrático (Grado 2)
a, b, c = np.polyfit(x_pixels, y_robot, 2)

print("\n================ COEFICIENTES OBTENIDOS ================")
print(f"a = {a:.10f}")
print(f"b = {b:.10f}")
print(f"c = {c:.10f}")
print(f"\nFórmula: Y_robot = ({a:.8f})*X_px^2 + ({b:.8f})*X_px + ({c:.8f})")
print("========================================================\n")
