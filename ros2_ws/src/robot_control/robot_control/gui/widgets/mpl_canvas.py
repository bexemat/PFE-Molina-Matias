import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MplCanvas(FigureCanvas):
    """Lienzo de Matplotlib modular e independiente para graficar las 3 articulaciones."""
    
    def __init__(self, parent=None, width=5, height=6, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes_q1 = fig.add_subplot(311)
        self.axes_q2 = fig.add_subplot(312)
        self.axes_q3 = fig.add_subplot(313)
        fig.tight_layout(pad=2.0)
        super().__init__(fig)