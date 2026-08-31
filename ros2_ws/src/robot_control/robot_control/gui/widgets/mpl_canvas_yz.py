from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MplCanvas2D_YZ(FigureCanvas):
    """Lienzo Matplotlib 2D para graficar el plano Y-Z."""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)

    def plot_yz(self, start_y, start_z, target_y, target_z, trace_y, trace_z):
        self.ax.cla()
        
        # Trayectoria dinámica
        if len(trace_y) > 0:
            self.ax.plot(trace_y, trace_z, 'k-', label='Trayectoria', linewidth=2)
            
        # Punto Inicial (Azul)
        if start_y is not None and start_z is not None:
            self.ax.scatter([start_y], [start_z], color='blue', s=80, label='Punto Inicial')
            
        # Punto Final / Objetivo (Verde)
        if target_y is not None and target_z is not None:
            self.ax.scatter([target_y], [target_z], color='green', s=80, label='Punto Final')

        self.ax.set_xlabel('Coordenada Y [mm]')
        self.ax.set_ylabel('Coordenada Z [mm]')
        self.ax.set_title('Gráfico Y-Z')
        self.ax.legend(loc='upper right', fontsize=8)
        self.ax.grid(True)
        self.draw()