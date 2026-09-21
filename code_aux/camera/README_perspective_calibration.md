# 🎯 Calibración píxel → mundo real (homografía)

Este script es el **segundo paso** de tu pipeline, justo después de `camera_calibration.py`. Mientras aquel corrige la **distorsión del lente** (ojo de pez), este convierte una posición en **píxeles** de la imagen ya corregida a una posición **real** (en milímetros) sobre el plano de trabajo donde estarán los pistachos — el dato que finalmente le pasarás al brazo robótico.

> Se calibra una sola vez, con la cámara ya **montada en su posición definitiva**. Si mueves la cámara, hay que repetirla.

---

## 🧠 ¿Por qué hace falta esto además de `camera_calibration.py`?

- `camera_calibration.py` corrige la **geometría del lente**: convierte una imagen curva en una imagen recta. El resultado sigue estando en **píxeles**.
- `perspective_calibration.py` responde a una pregunta distinta: *"el pistacho está en el píxel (450, 320)... ¿a cuántos milímetros reales corresponde eso sobre la mesa?"*. El resultado es la posición **real** que el brazo puede entender.

Sin este paso, tu detector de YOLO podría decirte perfectamente "el pistacho está en el píxel (450, 320)", pero el brazo robótico no sabe qué significa eso en su propio espacio de trabajo.

---

## ⚙️ Funciones del script

### `loadCameraCalibration(paramPath=None)`
Carga `camMatrix` y `distCoeff` desde el `calibration.npz` que generó `camera_calibration.py`. Si no existe ese archivo, lanza un error indicándote que corras primero ese script.

### `captureReferenceImage(camIndex=0, savePath=None)`
Abre la cámara en vivo para que tomes una foto de la superficie de trabajo con tus puntos de referencia visibles (presiona `c` para capturar, `q` para cancelar). Es opcional: solo la necesitas si prefieres tomar la foto en el momento en vez de usar una ya guardada.

> ⚠️ Esta función usa `cv.VideoCapture()` con los ajustes por defecto de OpenCV. Si tus imágenes de calibración de lente (las que usaste en `camera_calibration.py`) vinieron de un **pipeline de captura distinto** —por ejemplo, una app propia que guarda `.npy` y luego lo conviertes a `.jpg`—, `camMatrix`/`distCoeff` quedaron calculados para ese pipeline, no para `cv.VideoCapture()`. Usarlos sobre una imagen capturada aquí puede dar una corrección desalineada (la imagen sale **más** distorsionada, no menos). En ese caso, usa la Opción A: genera la imagen de referencia con el mismo pipeline que usaste para calibrar el lente.

### `selectPixelPoints(img, nPoints)`
Muestra la imagen y te deja hacer **clic con el mouse**, en orden, sobre los puntos de referencia (por ejemplo las 4 esquinas de un rectángulo marcado en la mesa). Devuelve las coordenadas en píxeles de cada clic. La ventana se puede **redimensionar o maximizar** arrastrando sus bordes para marcar los puntos con más precisión — esto no afecta el resultado, OpenCV traduce automáticamente el clic al píxel real de la imagen, sea cual sea el tamaño de la ventana.

### `computeHomography(pixelPoints, worldPoints)`
Con los puntos en píxeles que marcaste y sus coordenadas reales conocidas (`WORLD_POINTS_MM`), calcula la **matriz de homografía** con `cv.findHomography()`. Esta matriz es la que traduce cualquier punto de la imagen a coordenadas reales.

### `pixelToWorld(H, px, py)`
Dada la homografía ya calculada y un punto en píxeles, devuelve su posición real `(X, Y)` en milímetros. **Esta es la función que usarás dentro de tu detector final con YOLO.**

### `testHomography(img, H)`
Modo de verificación: haz clic en cualquier parte de la imagen y en la consola se imprime la posición real correspondiente. Sirve para comprobar que todo quedó bien calibrado antes de confiar en el sistema.

### `runPerspectiveCalibration(refImagePath=None, useCamera=False, camIndex=0)`
Función principal que encadena todo el proceso:
1. Carga `camMatrix`/`distCoeff`.
2. Obtiene la imagen de referencia (de archivo o de la cámara).
3. La des-distorsiona con los parámetros del primer script.
4. Te deja marcar los puntos de referencia.
5. Calcula y guarda la homografía en `homography.npz`.
6. Abre el modo de prueba para verificar visualmente.

---

## 🔧 Configuración: `WORLD_POINTS_MM`

Al principio del script hay una lista que **debes editar tú** con las coordenadas reales de tus puntos de referencia:

```python
WORLD_POINTS_MM = np.array([
    [0,   0],    # esquina superior izquierda
    [200, 0],    # esquina superior derecha
    [200, 150],  # esquina inferior derecha
    [0,   150],  # esquina inferior izquierda
], dtype=np.float32)
```

Esto representa un rectángulo real de 200 x 150 mm en tu superficie de trabajo, con el origen (0,0) donde tú decidas (por ejemplo, la esquina donde luego el brazo tiene su propio origen, para simplificar la siguiente conversión).

**Puntos de referencia recomendados:**
- Una hoja de papel o cartulina de tamaño conocido, pegada sobre la superficie.
- Una plantilla impresa con 4 marcas en posiciones medidas.
- 4 puntos marcados con cinta métrica directamente sobre la mesa o cinta transportadora.

> ⚠️ El orden en que escribas los puntos en `WORLD_POINTS_MM` debe **corresponder uno a uno** con el orden en que los vas a hacer clic sobre la imagen: tu primer clic = la primera fila de `WORLD_POINTS_MM`, el segundo clic = la segunda fila, y así sucesivamente. El punto de partida y el sentido (empezar por la esquina que quieras, ir en sentido horario o antihorario) **no importan** — lo único que importa es que ambas listas estén en el mismo orden entre sí.

---

## 📁 Estructura de carpetas esperada

```
tu_proyecto/
├── camera_calibration.py
├── calibration.npz              # generado por camera_calibration.py
├── perspective_calibration.py
├── homography.npz               # generado por este script
└── demoImages/
    ├── calibration/              # fotos del tablero (paso 1)
    └── perspective/
        └── reference.jpg         # foto de la superficie de trabajo (paso 2)
```

---

## ▶️ Cómo usarlo

### 1. Ten listo `calibration.npz`
Este script depende de haber corrido antes `camera_calibration.py`.

### 2. Prepara tus puntos de referencia físicos
Coloca sobre tu superficie de trabajo (donde estarán los pistachos) algo con posiciones **conocidas y medibles**: una hoja de tamaño estándar, una plantilla impresa, o marcas medidas con regla/cinta métrica.

### 3. Edita `WORLD_POINTS_MM`
Reemplaza los valores de ejemplo por las medidas reales de tus puntos, en el mismo orden en que los vas a marcar.

### 4. Consigue la imagen de referencia
La imagen debe ser la foto **cruda, con distorsión** (ojo de pez) tal cual la entrega la cámara — no la corrijas tú a mano, el script aplica `cv.undistort()` internamente.

Dos opciones:
- **Opción A (recomendada): con una foto ya tomada.** Genera la imagen con el **mismo pipeline de captura** que usaste para las fotos de calibración de lente (por ejemplo, tu propia app que guarda `.npy` y luego lo conviertes a `.jpg`). Guárdala en `demoImages/perspective/reference.jpg`.
- **Opción B: con la cámara en vivo (`cv.VideoCapture`).** En la última línea del script, comenta `runPerspectiveCalibration(useCamera=False)` y descomenta la línea con `useCamera=True`. Solo úsala si tu pipeline real de captura es exactamente ese `cv.VideoCapture` (ver advertencia en `captureReferenceImage()` más arriba) — si tus fotos de calibración vinieron de un pipeline distinto, esta opción dará una corrección desalineada.

### 5. Ejecuta el script
```bash
python perspective_calibration.py
```
Se abrirá la imagen ya des-distorsionada (el script corrige la distorsión internamente antes de mostrarla) en una ventana que puedes **redimensionar o maximizar** arrastrando sus bordes. Haz clic, **en orden**, sobre cada uno de tus puntos de referencia.

### 6. Verifica el resultado
Automáticamente se abre el modo de prueba: haz clic en distintas partes de la imagen y compara en consola si la posición real impresa tiene sentido (por ejemplo, si haces clic cerca del centro de tu rectángulo de 200x150 mm, deberías ver algo cercano a 100, 75).

### 7. Ya tienes `homography.npz`
Contiene la matriz `H` lista para usar en tu detector final.

---

## 🔗 Cómo se conecta con tu detector YOLO

En tu script final (`pistachio_detector.py`), cargas ambos archivos una sola vez al iniciar, y por cada frame en tiempo real aplicas primero la corrección de lente y luego la de perspectiva:

```python
import numpy as np
import cv2 as cv

calib = np.load('calibration.npz')
persp = np.load('homography.npz')
camMatrix, distCoeff = calib['camMatrix'], calib['distCoeff']
H = persp['homography']

# --- por cada frame de la cámara en vivo ---
frameUndist = cv.undistort(frame, camMatrix, distCoeff, None, camMatrix)

# ... aquí corres YOLO sobre frameUndist y obtienes el centro del
# pistacho detectado en píxeles: (px, py) ...

puntoPixel = np.array([[[px, py]]], dtype=np.float32)
puntoReal = cv.perspectiveTransform(puntoPixel, H)
X, Y = puntoReal[0][0]  # coordenadas reales en mm, listas para el brazo
```

---

## ✅ Checklist antes de pasar al detector

- [ ] `calibration.npz` existe y viene de una calibración de lente con buen error de reproyección
- [ ] La cámara ya está en su posición **final y fija** (si la mueves, repite este paso)
- [ ] `WORLD_POINTS_MM` refleja las medidas reales de tus puntos de referencia
- [ ] Marqué los puntos en la imagen en el mismo orden que en `WORLD_POINTS_MM`
- [ ] Probé `testHomography()` y las coordenadas reales impresas tienen sentido físico
- [ ] `homography.npz` se generó correctamente

---

## 💡 Notas

- Cuantos más puntos de referencia uses (más de 4, bien distribuidos por toda el área de trabajo), más robusta será la homografía. Con 4 puntos alcanza si están en las esquinas del área útil, pero si notas errores grandes en los bordes, prueba con 6-8 puntos.
- Esta homografía asume que el pistacho está siempre sobre el **mismo plano** que usaste para calibrar (la mesa, la cinta). Si el pistacho tiene una altura variable relevante para tu brazo, este método plano no la captura — necesitarías profundidad (por ejemplo con una cámara estéreo o de profundidad).
- El origen (0,0) que definas en `WORLD_POINTS_MM` conviene que coincida con un punto de referencia fácil de trasladar al sistema de coordenadas del brazo robótico, para que la siguiente conversión sea lo más simple posible.
- **Usa siempre el mismo pipeline de captura** en las tres etapas: las fotos del tablero (`camera_calibration.py`), la imagen de referencia (este script) y la captura en tiempo real de tu detector final. Si alguna de las tres usa una fuente distinta (por ejemplo, tu app Qt en unas y `cv.VideoCapture` directo en otra), `camMatrix`/`distCoeff` dejan de corresponder a esa imagen y la corrección de distorsión sale mal.
