"""Lienzo de graficado en tiempo real de las variables articulares en Matplotlib."""

from typing import Optional
from PyQt5.QtWidgets import QWidget
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MplCanvas(FigureCanvas):
    """Lienzo con 3 subgráficos para telemetría angular (Q1, Q2, Q3)."""

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        width: float = 5.0,
        height: float = 6.0,
        dpi: int = 100,
    ) -> None:
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes_q1 = fig.add_subplot(311)
        self.axes_q2 = fig.add_subplot(312)
        self.axes_q3 = fig.add_subplot(313)
        fig.tight_layout(pad=2.0)
        super().__init__(fig)