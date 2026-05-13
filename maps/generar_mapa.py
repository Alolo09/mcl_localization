import cv2
import numpy as np
import yaml

# Parámetros del mundo (deben coincidir con mi_mundo.world)
resolution  = 0.05      # metros por píxel
map_size    = 200       # 200x200 px = 10x10 metros
origin_x    = map_size // 2
origin_y    = map_size // 2

# Mapa blanco = espacio libre
mapa = np.full((map_size, map_size), 255, dtype=np.uint8)

def metros_a_px(x, y):
    px = int(origin_x + x / resolution)
    py = int(origin_y - y / resolution)
    return px, py

def dibujar_caja(mapa, cx, cy, ancho, alto, grosor_px=2):
    # Esquinas en metros
    x1, y1 = metros_a_px(cx - ancho/2, cy + alto/2)
    x2, y2 = metros_a_px(cx + ancho/2, cy - alto/2)
    cv2.rectangle(mapa, (x1, y1), (x2, y2), 0, -1)  # -1 = relleno

# -- Paredes (igual que en mi_mundo.world) --
dibujar_caja(mapa,  0,    5,   10,  0.2)   # Norte
dibujar_caja(mapa,  0,   -5,   10,  0.2)   # Sur
dibujar_caja(mapa,  5,    0,   0.2, 10)    # Este
dibujar_caja(mapa, -5,    0,   0.2, 10)    # Oeste

# -- Obstáculos (igual que en mi_mundo.world) --
dibujar_caja(mapa,  2,  2,  1, 1)          # Obstáculo 1
dibujar_caja(mapa, -2, -2,  1, 1)          # Obstáculo 2
dibujar_caja(mapa, -2,  3,  1, 2)          # Obstáculo 3

# Guardar mapa
cv2.imwrite('mi_mapa.pgm', mapa)
print('Mapa guardado: mi_mapa.pgm')

# Guardar metadata
meta = {
    'image':           'mi_mapa.pgm',
    'resolution':      resolution,
    'origin': [-5.0, -5.0, 0.0],
    'negate':          0,
    'occupied_thresh': 0.65,
    'free_thresh':     0.196,
}
with open('mi_mapa.yaml', 'w') as f:
    yaml.dump(meta, f)
print('Metadata guardada: mi_mapa.yaml')

# Mostrar el mapa
cv2.imshow('Mapa conocido', mapa)
cv2.waitKey(0)
cv2.destroyAllWindows()
