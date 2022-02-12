import math

def lwr_figure_eight(t):
    return [0.6, -0.1 + math.sin(t * 2.0 * math.pi * 0.5) * 0.1, 0.5 + math.sin(t * math.pi * 0.5) * 0.2, 0.0, 0.0, 0.0, 1.0]
