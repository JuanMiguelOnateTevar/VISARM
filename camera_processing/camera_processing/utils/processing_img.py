import os
import numpy as np
import cv2 as cv
from ultralytics import YOLO

# =====================================================================
# CONFIGURACIÓN
# =====================================================================
CUR_FOLDER = os.path.dirname(os.path.abspath(__file__))

# Rutas a los parámetros generados por los scripts de calibración
CALIBRATION_PATH = os.path.join(CUR_FOLDER, 'calibration.npz')
HOMOGRAPHY_PATH = os.path.join(CUR_FOLDER, 'homography.npz')

# Ruta a tu modelo YOLO entrenado (ajusta el nombre al tuyo)
YOLO_MODEL_PATH = os.path.join(CUR_FOLDER, 'best.pt')

# Confianza mínima para aceptar una detección; por debajo de esto se
# descarta y se trata como si no se hubiera detectado nada
CONF_THRESHOLD = 0.5

# Nombres de las clases tal como las definiste al entrenar el modelo.
# Deben coincidir EXACTAMENTE (mayúsculas/minúsculas incluidas) con
# model.names, o la comparación de más abajo no funcionará.
CLASS_OK = 'ok'
CLASS_NOK = 'nok'

# Colores en formato BGR (el que usa OpenCV, no RGB)
COLOR_OK = (0, 200, 0)     # verde
COLOR_NOK = (0, 0, 255)    # rojo


# =====================================================================
# CARGA DE RECURSOS (se hace UNA sola vez, al importar este módulo)
# =====================================================================
# Cargar parámetros de calibración de lente
_calib = np.load(CALIBRATION_PATH)
CAM_MATRIX = _calib['camMatrix']
DIST_COEFF = _calib['distCoeff']

# Cargar homografía píxel -> mundo real
_persp = np.load(HOMOGRAPHY_PATH)
HOMOGRAPHY = _persp['homography']

# Cargar el modelo YOLO (Ultralytics). Esto es lo más "pesado" de
# cargar, por eso se hace una sola vez aquí y no dentro de la función
# que se llama por cada frame.
_model = YOLO(YOLO_MODEL_PATH)


# =====================================================================
# FUNCIONES AUXILIARES
# =====================================================================
def _pixel_to_world(px: float, py: float) -> tuple[float, float]:
    """
    Convierte un punto en píxeles (de la imagen ya des-distorsionada)
    a coordenadas reales en milímetros sobre el plano de trabajo,
    usando la homografía cargada de homography.npz.
    """
    puntoPixel = np.array([[[px, py]]], dtype=np.float32)
    puntoMundo = cv.perspectiveTransform(puntoPixel, HOMOGRAPHY)
    x, y = puntoMundo[0][0]
    return float(x), float(y)


def _draw_detection(frame: np.ndarray, x1: float, y1: float, x2: float,
                     y2: float, color: tuple[int, int, int],
                     label: str) -> None:
    """
    Dibuja la caja delimitadora y una etiqueta con fondo, sobre el
    frame, en el color indicado (verde para OK, rojo para NOK).
    Modifica 'frame' directamente (no devuelve nada).
    """
    p1 = (int(x1), int(y1))
    p2 = (int(x2), int(y2))
    cv.rectangle(frame, p1, p2, color, 2)

    (textW, textH), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    # Fondo sólido detrás del texto para que se lea bien sobre cualquier imagen
    cv.rectangle(frame, (p1[0], p1[1] - textH - 10),
                 (p1[0] + textW + 8, p1[1]), color, -1)
    cv.putText(frame, label, (p1[0] + 4, p1[1] - 6),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


# =====================================================================
# FUNCIÓN PRINCIPAL
# =====================================================================
def processing_img(frame_raw: np.ndarray) -> tuple[np.ndarray, str, dict[str, float]]:
    """
    Procesa un frame crudo de la cámara de principio a fin:

    1) Detecta el pistacho con YOLO directamente sobre el frame CRUDO
       (clases 'ok' / 'nok'). No se corrige la distorsión de toda la
       imagen: así el modelo ve la misma apariencia que en el
       entrenamiento, sin bordes negros ni esquinas deformadas.
    2) Corrige SOLO el punto central de la detección con
       cv.undistortPoints() (mucho más barato que corregir la imagen
       completa, y evita el problema de las esquinas deformadas).
    3) Convierte ese punto ya corregido a coordenadas reales (mm)
       usando homography.npz
    4) Dibuja sobre el frame la caja delimitadora:
       - VERDE si la clase detectada es 'ok', con fiabilidad y coordenada
       - ROJO si la clase detectada es 'nok', con fiabilidad y coordenada
    5) Devuelve el frame procesado, un mensaje de estado y la posición

    Parámetros:
    - frame_raw: frame crudo (CON distorsión), tal como llega de la
      cámara. Tus imágenes de entrenamiento de YOLO deben ser
      igualmente crudas, sin pasar por camera_calibration.py.

    Devuelve (misma forma que proccesing_img_simu, pero con datos reales):
    - frame_pro: el mismo frame recibido, con la caja y el texto
      dibujados encima (no se modifica su geometría)
    - message: 'OK', 'NOK' o 'NO_DETECTION' si no se detectó nada con
      confianza suficiente
    - pose: diccionario {'x': ..., 'y': ...} con la posición real en
      mm del objeto detectado. Si no hubo detección, se devuelve {}.
    """
    # 1) YOLO corre directamente sobre el frame CRUDO (sin corregir la
    # imagen completa). Así el modelo ve exactamente la misma
    # apariencia que tuvo en las imágenes de entrenamiento -sin bordes
    # negros ni esquinas deformadas por cv.undistort()-, y nos
    # ahorramos procesar la imagen entera en cada frame.
    results = _model.predict(frame_raw, conf=CONF_THRESHOLD, verbose=False)[0]
    frame_pro = frame_raw

    if len(results.boxes) == 0:
        return frame_pro, 'NO_DETECTION', {}

    # Si hay varias detecciones, nos quedamos con la de mayor confianza.
    # (Si tu caso necesita procesar VARIAS a la vez, dímelo y adaptamos
    # esta parte para devolver una lista en vez de un solo resultado.)
    confidences = results.boxes.conf.cpu().numpy()
    bestIdx = int(np.argmax(confidences))
    box = results.boxes[bestIdx]

    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
    conf = float(box.conf[0].cpu().numpy())
    clsId = int(box.cls[0].cpu().numpy())
    className = _model.names[clsId]

    # 2) Centro de la caja, en píxeles del frame CRUDO (con distorsión)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

    # 3) Corregir SOLO ese punto (no toda la imagen). P=camMatrixNew
    # para que quede en el mismo espacio de píxeles que usó
    # perspective_calibration.py al calcular la homografía (esa
    # calibración se hizo sobre una imagen des-distorsionada con ese
    # mismo camMatrixNew, así que deben coincidir).
    height, width = frame_raw.shape[:2]
    camMatrixNew, _ = cv.getOptimalNewCameraMatrix(
        CAM_MATRIX, DIST_COEFF, (width, height), 1, (width, height))
    puntoDistorsionado = np.array([[[cx, cy]]], dtype=np.float32)
    puntoCorregido = cv.undistortPoints(
        puntoDistorsionado, CAM_MATRIX, DIST_COEFF, P=camMatrixNew)
    cxUndist, cyUndist = puntoCorregido[0][0]

    # 4) Ese punto ya corregido es el que se convierte a mm con la homografía
    worldX, worldY = _pixel_to_world(cxUndist, cyUndist)
    pose = {'x': worldX, 'y': worldY}

    # 5) Determinar OK/NOK y dibujar (la caja se dibuja en coordenadas
    # del frame crudo, que es sobre el que estamos mostrando la imagen;
    # solo la posición real usada para el brazo pasó por la corrección)
    isOk = className.lower() == CLASS_OK.lower()
    message = 'OK' if isOk else 'NOK'
    color = COLOR_OK if isOk else COLOR_NOK

    label = f"{message} {conf * 100:.1f}%  ({worldX:.1f}, {worldY:.1f}) mm"
    _draw_detection(frame_pro, x1, y1, x2, y2, color, label)

    return frame_pro, message, pose


# =====================================================================
# Versión simulada original (útil para probar el resto del pipeline
# -por ejemplo el envío de posición al brazo- sin depender de la
# cámara ni del modelo YOLO real)
# =====================================================================
def proccesing_img_simu(frame_raw: np.ndarray) -> tuple[np.ndarray, str, dict[str, float]]:
    frame_pro = frame_raw
    message = 'OK'
    pose = {'x': 100.0, 'y': 100.0}
    return frame_pro, message, pose


# =====================================================================
# Prueba rápida con la webcam
# =====================================================================
if __name__ == '__main__':
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la cámara.")

    print("Presiona 'q' para salir.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame_pro, message, pose = processing_img(frame)
        print(f"Mensaje: {message}  |  Posición: {pose}")

        cv.imshow('Detección', frame_pro)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()