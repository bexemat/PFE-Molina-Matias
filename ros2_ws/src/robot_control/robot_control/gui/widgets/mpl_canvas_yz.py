"""Lienzo bidimensional en Matplotlib para la proyección del plano sagital Y-Z."""

from typing import List, Optional
from PyQt5.QtWidgets import QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MplCanvas2D_YZ(FigureCanvas):
    """Lienzo gráfico para monitorear la trayectoria real del efector en Y-Z [mm]."""

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        width: float = 5.0,
        height: float = 4.0,
        dpi: int = 100,
    ) -> None:
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)

    def plot_yz(
        self,
        start_y: Optional[float],
        start_z: Optional[float],
        target_y: Optional[float],
        target_z: Optional[float],
        trace_y: List[float],
        trace_z: List[float],
    ) -> None:
        """Renderiza la trayectoria recorrida respecto a los puntos objetivo.

        Args:
            start_y (Optional[float]): Coordenada Y de inicio [mm].
            start_z (Optional[float]): Coordenada Z de inicio [mm].
            target_y (Optional[float]): Coordenada Y final deseada [mm].
            target_z (Optional[float]): Coordenada Z final deseada [mm].
            trace_y (List[float]): Historial de posiciones Y reales [mm].
            trace_z (List[float]): Historial de posiciones Z reales [mm].
        """
        self.ax.cla()

        if len(trace_y) > 0:
            self.ax.plot(trace_y, trace_z, "k-", label="Trayectoria", linewidth=2)

        if start_y is not None and start_z is not None:
            self.ax.scatter(
                [start_y], [start_z], color="blue", s=80, label="Punto Inicial"
            )

        if target_y is not None and target_z is not None:
            self.ax.scatter(
                [target_y], [target_z], color="green", s=80, label="Punto Final"
            )

        self.ax.set_xlabel("Coordenada Y [mm]")
        self.ax.set_ylabel("Coordenada Z [mm]")
        self.ax.set_title("Gráfico Sagital Y-Z")
        self.ax.legend(loc="upper right", fontsize=8)
        self.ax.grid(True)
        self.draw()