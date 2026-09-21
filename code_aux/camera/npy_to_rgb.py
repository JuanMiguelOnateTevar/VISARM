import os
import numpy as np
from PIL import Image

# Carpeta donde están los archivos .npy
npy_folder = "/home/juanmi/Desktop/NOK"

# Carpeta donde se van a guardar los archivos .jpg
jpg_folder = "/home/juanmi/curso_ros2_ws/src/VISARM/code_aux/camera/img_rgb/img_test"

for filename in os.listdir(npy_folder):

    if filename.endswith(".npy"):

        # Cargar imagen
        npy_path = os.path.join(npy_folder, filename)
        img = np.load(npy_path)

        # Normalizar a 0-255 si no está ya en uint8
        if img.dtype != np.uint8:
            img = img.astype(np.float32)

            img = img - img.min()

            if img.max() > 0:
                img = img / img.max()

            img = (img * 255).astype(np.uint8)

        # BGR -> RGB
        if img.ndim == 3 and img.shape[2] == 3:
            img = img[:, :, ::-1]

        # Crear nombre .jpg
        jpg_name = os.path.splitext(filename)[0] + ".jpg"
        jpg_path = os.path.join(jpg_folder, jpg_name)

        # Guardar
        Image.fromarray(img).save(jpg_path, quality=95)

        print(f"{filename} -> {jpg_name}")

print("Conversión terminada.")