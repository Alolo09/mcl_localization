import cv2, yaml, numpy as np

def load_map(pgm_path, yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    resolution = meta['resolution']      # ej. 0.05 m/px
    origin     = meta['origin']          # [x, y, θ] en metros
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    return img, resolution, origin