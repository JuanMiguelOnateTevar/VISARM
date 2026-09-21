import numpy as np
import cv2 as cv
import os

# =====================================================================
# CONFIGURA AQUÍ tus puntos de referencia en el MUNDO REAL
# =====================================================================
# Son las coordenadas reales (en milímetros, sobre el plano de trabajo
# donde estarán los pistachos) de 4 o más puntos que puedas señalar
# físicamente: por ejemplo las 4 esquinas de una hoja, de una plantilla
# impresa, o marcas que pegues sobre la mesa/cinta transportadora.
#
# El orden en que los escribas aquí DEBE coincidir con el orden en que
# los vas a hacer clic sobre la imagen (ver selectPixelPoints()).
#
# Ejemplo: un rectángulo de 200 x 150 mm, con el origen (0,0) en la
# esquina superior izquierda de tu área de trabajo.
# WORLD_POINTS_MM = np.array([
#     [0,   0],    # esquina superior izquierda
#     [200, 0],    # esquina superior derecha
#     [200, 150],  # esquina inferior derecha
#     [0,   150],  # esquina inferior izquierda
# ], dtype=np.float32)
WORLD_POINTS_MM = np.array([
    [0,   0],    # esquina superior izquierda, esta primera esquina sera nuestro 0,0 sera nuestro eje de cordenadas a la hora de dar posiciones.
    [106, 0],    # esquina superior derecha
    [106, 74],  # esquina inferior derecha
    [0,   74],  # esquina inferior izquierda
], dtype=np.float32)


def loadCameraCalibration(paramPath=None):
    """
    Carga camMatrix y distCoeff generados por camera_calibration.py.

    Parámetros:
    - paramPath: ruta al archivo calibration.npz. Si no se indica, se
      busca junto a este mismo script.

    Devuelve:
    - camMatrix, distCoeff
    """
    if paramPath is None:
        curFolder = os.path.dirname(os.path.abspath(__file__))
        paramPath = os.path.join(curFolder, 'calibration.npz')

    if not os.path.exists(paramPath):
        raise FileNotFoundError(
            f"No se encontró '{paramPath}'. Corre primero "
            "camera_calibration.py para generarlo.")

    data = np.load(paramPath)
    return data['camMatrix'], data['distCoeff']


def captureReferenceImage(camIndex=0, savePath=None):
    """
    Abre la cámara en vivo y, al presionar la tecla 'c', captura una
    foto de referencia (la superficie de trabajo con tus puntos/marcas
    visibles). Presiona 'q' para salir sin capturar.

    ⚠️ IMPORTANTE: esta función usa cv.VideoCapture() directamente, con
    los ajustes por defecto de OpenCV (resolución, formato, etc.). Si
    tus imágenes de calibración de lente (las que usaste en
    camera_calibration.py) vinieron de un pipeline distinto -por
    ejemplo, una app propia que captura .npy y luego convierte a .jpg,
    como una app Qt-, camMatrix/distCoeff quedaron calculados para ESE
    pipeline, no para cv.VideoCapture(). Usarlos sobre una imagen
    capturada aquí puede dar una corrección desalineada (la imagen se
    ve MÁS distorsionada, no menos).

    Usa esta función solo si TODO tu flujo (calibración de lente, foto
    de referencia y detector final) captura las imágenes de la misma
    forma. Si no es así, ignora esta función y usa la Opción A: genera
    la imagen de referencia con tu propio pipeline de captura (el
    mismo que usaste para las fotos del tablero) y pásasela al script
    ya guardada como archivo.

    Úsala si prefieres tomar la foto en el momento en vez de usar una
    imagen ya guardada.

    Parámetros:
    - camIndex: índice de la cámara (0 = cámara por defecto)
    - savePath: dónde guardar la imagen capturada. Si no se indica, se
      guarda en 'demoImages/perspective/reference.jpg'

    Devuelve:
    - La imagen capturada (array de OpenCV) o None si se canceló
    """
    if savePath is None:
        root = os.getcwd()
        refDir = os.path.join(root, 'demoImages', 'perspective')
        os.makedirs(refDir, exist_ok=True)
        savePath = os.path.join(refDir, 'reference.jpg')

    cap = cv.VideoCapture(camIndex)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la cámara.")

    print("Presiona 'c' para capturar la foto de referencia, 'q' para salir.")
    frame = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv.imshow('Presiona c para capturar', frame)
        key = cv.waitKey(1) & 0xFF
        if key == ord('c'):
            cv.imwrite(savePath, frame)
            print(f"Imagen guardada en: {savePath}")
            break
        elif key == ord('q'):
            frame = None
            break

    cap.release()
    cv.destroyAllWindows()
    return frame


def selectPixelPoints(img, nPoints):
    """
    Muestra la imagen y permite hacer clic con el mouse para marcar,
    EN ORDEN, los mismos puntos que definiste en WORLD_POINTS_MM.

    Ejemplo: si WORLD_POINTS_MM tiene las 4 esquinas de un rectángulo
    (superior-izq, superior-der, inferior-der, inferior-izq), debes
    hacer clic sobre la imagen respetando ese mismo orden.

    Parámetros:
    - img: imagen (ya des-distorsionada) donde se ven los puntos de
      referencia
    - nPoints: cuántos puntos se esperan marcar

    Devuelve:
    - Array de puntos en píxeles, shape (nPoints, 2)
    """
    pixelPoints = []
    imgDisplay = img.copy()
    winName = 'Selecciona los puntos en orden'

    def onClick(event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN and len(pixelPoints) < nPoints:
            pixelPoints.append([x, y])
            cv.circle(imgDisplay, (x, y), 6, (0, 0, 255), -1)
            cv.putText(imgDisplay, str(len(pixelPoints)), (x + 10, y),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv.imshow(winName, imgDisplay)

    # WINDOW_NORMAL permite redimensionar/maximizar la ventana con el
    # mouse (por defecto, cv.imshow() crea una ventana de tamaño fijo
    # igual a la resolución de la imagen, que en imágenes pequeñas es
    # incómoda para marcar puntos con precisión). Redimensionar la
    # ventana NO afecta el resultado: OpenCV traduce automáticamente
    # las coordenadas del clic al tamaño real de la imagen.
    cv.namedWindow(winName, cv.WINDOW_NORMAL)
    cv.resizeWindow(winName, 1280, 800)
    cv.imshow(winName, imgDisplay)
    cv.setMouseCallback(winName, onClick)

    print(f"Haz clic sobre los {nPoints} puntos de referencia, EN ORDEN.")
    print("Puedes arrastrar los bordes de la ventana o maximizarla para "
          "marcar los puntos con más precisión.")
    print("Presiona cualquier tecla cuando termines.")
    while len(pixelPoints) < nPoints:
        if cv.waitKey(20) & 0xFF != 255:
            break
    cv.waitKey(0)
    cv.destroyAllWindows()

    return np.array(pixelPoints, dtype=np.float32)


def computeHomography(pixelPoints, worldPoints=WORLD_POINTS_MM):
    """
    Calcula la matriz de homografía que transforma coordenadas de
    píxel (en la imagen ya des-distorsionada) a coordenadas reales
    del plano de trabajo (en mm).

    Parámetros:
    - pixelPoints: puntos en píxeles (de selectPixelPoints())
    - worldPoints: puntos reales correspondientes (WORLD_POINTS_MM)

    Devuelve:
    - H: matriz de homografía 3x3
    """
    H, status = cv.findHomography(pixelPoints, worldPoints)
    return H


def pixelToWorld(H, px, py):
    """
    Convierte un punto en píxeles (de la imagen ya des-distorsionada)
    a coordenadas reales (mm) usando la homografía calculada.

    Parámetros:
    - H: matriz de homografía (de computeHomography())
    - px, py: coordenadas del punto en píxeles

    Devuelve:
    - (X, Y): coordenadas reales en mm sobre el plano de trabajo
    """
    puntoPixel = np.array([[[px, py]]], dtype=np.float32)
    puntoMundo = cv.perspectiveTransform(puntoPixel, H)
    X, Y = puntoMundo[0][0]
    return float(X), float(Y)


def testHomography(img, H):
    """
    Modo de prueba: haz clic sobre la imagen des-distorsionada y verás
    impresas en consola las coordenadas reales (mm) correspondientes.
    Útil para verificar que la homografía quedó bien calculada antes
    de usarla en el detector final.

    Presiona 'q' para cerrar.
    """
    imgDisplay = img.copy()
    winName = 'Prueba: haz clic para ver coordenadas reales'

    def onClick(event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            X, Y = pixelToWorld(H, x, y)
            print(f"Píxel ({x}, {y})  ->  Mundo real ({X:.1f} mm, {Y:.1f} mm)")
            cv.circle(imgDisplay, (x, y), 5, (0, 255, 0), -1)
            cv.imshow(winName, imgDisplay)

    cv.namedWindow(winName, cv.WINDOW_NORMAL)
    cv.resizeWindow(winName, 1280, 800)
    cv.imshow(winName, imgDisplay)
    cv.setMouseCallback(winName, onClick)
    print("Haz clic en cualquier parte de la imagen para ver su posición real.")
    print("Presiona 'q' para salir.")
    while True:
        if cv.waitKey(20) & 0xFF == ord('q'):
            break
    cv.destroyAllWindows()


def runPerspectiveCalibration(refImagePath=None, useCamera=False, camIndex=0):
    """
    Flujo completo de calibración píxel-mundo:
    1) Carga camMatrix/distCoeff de camera_calibration.py
    2) Obtiene una imagen de referencia (de archivo o de la cámara)
    3) Des-distorsiona esa imagen con los parámetros ya calculados
    4) Deja que marques los puntos de referencia con el mouse
    5) Calcula la homografía y la guarda en 'homography.npz'
    6) Abre un modo de prueba para verificar el resultado

    Parámetros:
    - refImagePath: ruta a una imagen ya tomada de la superficie de
      trabajo con tus puntos de referencia visibles. Si es None y
      useCamera=True, se captura una nueva desde la cámara.
    - useCamera: si es True, ignora refImagePath y abre la cámara en
      vivo para capturar la foto de referencia.
    - camIndex: índice de cámara a usar si useCamera=True
    """
    camMatrix, distCoeff = loadCameraCalibration()

    # Obtener imagen de referencia
    if useCamera:
        img = captureReferenceImage(camIndex=camIndex)
        if img is None:
            print("Calibración cancelada.")
            return
    else:
        if refImagePath is None:
            root = os.getcwd()
            refImagePath = os.path.join(
                root, 'demoImages', 'perspective', 'reference.jpg')
        img = cv.imread(refImagePath)
        if img is None:
            raise FileNotFoundError(f"No se pudo leer la imagen: {refImagePath}")

    # Des-distorsionar ANTES de marcar los puntos: la homografía debe
    # calcularse sobre la geometría ya corregida, igual que se hará
    # después con cada frame del detector en tiempo real.
    height, width = img.shape[:2]
    camMatrixNew, roi = cv.getOptimalNewCameraMatrix(
        camMatrix, distCoeff, (width, height), 1, (width, height))
    imgUndist = cv.undistort(img, camMatrix, distCoeff, None, camMatrixNew)

    # Marcar los puntos de referencia en la imagen ya corregida
    nPoints = len(WORLD_POINTS_MM)
    pixelPoints = selectPixelPoints(imgUndist, nPoints)

    if len(pixelPoints) != nPoints:
        print("No se marcaron todos los puntos. Calibración cancelada.")
        return

    # Calcular y guardar la homografía
    H = computeHomography(pixelPoints, WORLD_POINTS_MM)
    print('Matriz de homografía:\n', H)

    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'homography.npz')
    np.savez(paramPath,
             homography=H,
             worldPoints=WORLD_POINTS_MM,
             pixelPoints=pixelPoints)
    print(f"Homografía guardada en: {paramPath}")

    # Modo de prueba para verificar visualmente
    testHomography(imgUndist, H)

    return H


if __name__ == '__main__':
    # IMPORTANTE: en ambas opciones la imagen de referencia debe ser la
    # foto CRUDA de la cámara, CON distorsión (ojo de pez) tal cual la
    # entrega la cámara. NO la corrijas tú a mano antes. El propio
    # script aplica cv.undistort() internamente usando camMatrix y
    # distCoeff de calibration.npz, así que la corrección ocurre sola.
    #
    # RECOMENDADO: usa la Opción A, generando la imagen con el MISMO
    # pipeline de captura que usaste para calibrar el lente (por
    # ejemplo, tu propia app que guarda .npy y luego lo conviertes a
    # .jpg). Si usas la Opción B (cv.VideoCapture directo) y tu
    # pipeline de calibración fue distinto, camMatrix/distCoeff no le
    # corresponden a esa imagen y la corrección sale mal.

    # Opción A: usar una imagen ya guardada (cruda, con distorsión) en
    # demoImages/perspective/reference.jpg
    runPerspectiveCalibration(useCamera=False)

    # Opción B: capturar la foto de referencia directamente desde la
    # cámara con cv.VideoCapture (también cruda, con distorsión); solo
    # úsala si tu pipeline real de captura es exactamente ese (ver
    # advertencia en captureReferenceImage())
    # runPerspectiveCalibration(useCamera=True, camIndex=0)