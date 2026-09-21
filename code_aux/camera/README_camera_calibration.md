# 📷 Calibración de cámara con OpenCV (tablero de ajedrez)

Este script calibra una cámara usando un patrón de **tablero de ajedrez** para obtener sus parámetros intrínsecos y sus coeficientes de distorsión. La idea final del proyecto es usar estos parámetros para **corregir la distorsión tipo "ojo de pez"** de la cámara antes de detectar un pistacho y enviar su posición a un brazo robótico. Si la imagen está distorsionada, la posición calculada del pistacho no coincidirá con su posición real, y el brazo apuntará mal.

> Basado en el video: https://www.youtube.com/watch?v=H5qbRTikxI4

---

## 🧠 ¿Por qué es necesario esto?

Las cámaras (sobre todo las de gran angular / ojo de pez) deforman las líneas rectas del mundo real, curvándolas cerca de los bordes de la imagen. Esa deformación se llama **distorsión**.

Para un sistema de visión que va a decirle a un brazo robótico "el pistacho está en la posición (x, y)", esa distorsión introduce un error de posición que crece cuanto más lejos del centro de la imagen esté el objeto. La calibración de cámara permite:

1. Medir matemáticamente esa distorsión (coeficientes de distorsión).
2. Conocer los parámetros internos de la cámara (matriz de cámara: distancia focal, punto principal).
3. Usar esos dos datos para **des-distorsionar** cualquier imagen futura, de modo que las coordenadas de píxel se correspondan con la realidad.

---

## ⚙️ Funciones del script

### `calibrate(showPics=True)`
Función principal de calibración.

**Qué hace paso a paso:**
1. Busca todas las imágenes `.jpg` dentro de `demoImages/calibration`. Estas deben ser varias fotos del **mismo** tablero de ajedrez, tomadas desde distintos ángulos y distancias (mientras más variedad, mejor calibración).
2. Para cada imagen, usa `cv.findChessboardCorners()` para detectar las esquinas internas del tablero.
3. Si las encuentra, refina su posición a nivel de subpíxel con `cv.cornerSubPix()` (más precisión = mejor calibración).
4. Junta todos esos puntos y llama a `cv.calibrateCamera()`, que devuelve:
   - **`camMatrix`**: matriz intrínseca de la cámara (foco y centro óptico).
   - **`distCoeff`**: coeficientes de distorsión (lo que corrige el ojo de pez).
   - **`repError`**: error de reproyección en píxeles (indica qué tan buena fue la calibración; **idealmente menor a 1**).
5. Guarda todo en un archivo `calibration.npz` para poder reutilizarlo sin repetir el proceso.

**Parámetro:**
- `showPics`: si es `True`, te muestra cada imagen con las esquinas detectadas dibujadas encima, para que verifiques visualmente que la detección fue correcta.

> ⚠️ **Corrección aplicada:** en el script original, `calibrate()` no devolvía nada, pero `runRemoveDistrosion()` la llama esperando `camMatrix` y `distCoeff` de vuelta. En esta versión se agregó `return camMatrix, distCoeff` al final para que el flujo completo funcione.

---

### `removeDistortion(camMatrix, distCoeff)`
Toma los parámetros obtenidos en `calibrate()` y los aplica sobre una imagen de prueba (`demoImages/distortion2.jpg`) para eliminar la distorsión.

**Qué hace:**
1. Calcula una matriz de cámara "óptima" con `cv.getOptimalNewCameraMatrix()`.
2. Aplica `cv.undistort()` para corregir la imagen.
3. Dibuja una línea de referencia sobre la imagen original y la corregida, para que puedas comparar visualmente cómo cambia la curvatura.
4. Muestra ambas imágenes lado a lado con `matplotlib`.

---

### `runCalibration()`
Atajo que simplemente llama a `calibrate(showPics=True)`. Úsalo cuando quieras **solo calibrar** y revisar visualmente que la detección de esquinas funcionó bien en tus fotos.

### `runRemoveDistrosion()`
Atajo que calibra (sin mostrar imágenes) y de inmediato aplica la corrección de distorsión sobre la imagen de prueba. Úsalo cuando ya confías en tus fotos de calibración y solo quieres ver el resultado final.

---

## 📁 Estructura de carpetas esperada

```
tu_proyecto/
├── camera_calibration.py
└── demoImages/
    ├── calibration/
    │   ├── foto1.jpg
    │   ├── foto2.jpg
    │   └── ... (10-20 fotos del tablero desde distintos ángulos)
    └── distortion2.jpg   (imagen de prueba para ver la corrección)
```

---

## ▶️ Cómo usarlo

### 1. Instala las dependencias
```bash
pip install opencv-python numpy matplotlib
```

### 2. Prepara tus imágenes de calibración
- Imprime o muestra en pantalla un **tablero de ajedrez** (patrón de calibración estándar).
- Con la cámara que vas a usar para detectar el pistacho, toma **10 a 20 fotos** del tablero desde ángulos y distancias distintas.
- Guárdalas todas en `demoImages/calibration/`.

> 🔧 El script está configurado para un tablero de **9x6 esquinas internas** (`nRows = 9`, `nCols = 6`). Si tu tablero tiene otra cantidad de cuadros, ajusta estos valores: el número de esquinas internas es siempre (cuadros_por_fila − 1) x (cuadros_por_columna − 1).

### 3. Ejecuta la calibración
```bash
python camera_calibration.py
```
Esto ejecuta `runCalibration()`, muestra cada imagen con las esquinas detectadas y al final imprime en consola la matriz de la cámara y el error de reproyección. También genera el archivo `calibration.npz` con todos los parámetros.

### 4. (Opcional) Verifica la corrección de distorsión
En la parte final del script, comenta `runCalibration()` y descomenta `runRemoveDistrosion()`:
```python
if __name__ == '__main__':
    # runCalibration()
    runRemoveDistrosion()
```
Esto te mostrará, lado a lado, la imagen original y la corregida.

### 5. Reutiliza los parámetros guardados
Para tu proyecto del brazo robótico, **no necesitas volver a calibrar cada vez**. Simplemente carga el archivo `calibration.npz`:
```python
import numpy as np

data = np.load('calibration.npz')
camMatrix = data['camMatrix']
distCoeff = data['distCoeff']
```
Y luego, en cada frame que captures de la cámara en tiempo real, aplica:
```python
import cv2 as cv

frameUndist = cv.undistort(frame, camMatrix, distCoeff, None, camMatrix)
```
A partir de ahí, la posición del pistacho que detectes en `frameUndist` estará libre de la deformación del ojo de pez, lista para convertirla en coordenadas reales y enviarla al brazo robótico.

---

## ✅ Checklist antes de pasar a la detección del pistacho

- [ ] Tomé al menos 10-15 fotos del tablero desde ángulos variados
- [ ] El error de reproyección (`repError`) es bajo (idealmente < 1 píxel)
- [ ] Verifiqué visualmente que las esquinas se detectaron bien en la mayoría de fotos
- [ ] Confirmé que `calibration.npz` se generó correctamente
- [ ] Probé `removeDistortion()` y la línea de referencia se ve recta en la imagen corregida

---

## 💡 Notas y siguientes pasos para tu proyecto del brazo robótico

- Una vez calibrada la cámara, el siguiente paso típico es hacer una **calibración cámara–brazo (hand-eye calibration)**, para convertir coordenadas de píxel en coordenadas del sistema de referencia del brazo robótico.
- Si vas a detectar el pistacho en tiempo real (video), aplica `cv.undistort()` a cada frame **antes** de correr tu algoritmo de detección (color, contornos, un modelo de detección de objetos, etc.).
- Guarda siempre las fotos de calibración originales: si cambias de lente o de resolución de cámara, tendrás que repetir este proceso.
