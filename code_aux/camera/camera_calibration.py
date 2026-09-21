import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt

def calibrate(showPics=True):
    """
    Calibra la cámara usando un patrón de tablero de ajedrez (chessboard).

    Qué hace:
    1) Busca todas las imágenes .jpg de la carpeta 'demoImages/calibration'.
    2) En cada imagen intenta detectar las esquinas internas del tablero.
    3) Con esas esquinas calcula los parámetros internos de la cámara
       (matriz de la cámara y coeficientes de distorsión).
    4) Guarda esos parámetros en un archivo 'calibration.npz' para poder
       reutilizarlos después sin tener que repetir la calibración.

    Parámetros:
    - showPics (bool): si es True, muestra cada imagen con las esquinas
      del tablero dibujadas encima (útil para verificar que la detección
      fue correcta).

    Devuelve:
    - camMatrix: matriz intrínseca de la cámara (foco, centro óptico, etc.)
    - distCoeff: coeficientes de distorsión (incluye el efecto "ojo de pez")
    """
    # Leer imágenes
    # root apunta a la carpeta donde se ejecuta el script
    root = os.getcwd()
    # Carpeta donde deben estar las fotos del tablero de ajedrez
    calibrationDir = os.path.join(root, 'img_rgb')

    # Lista con las rutas de todas las imágenes .jpg dentro de esa carpeta
    imgPathList = glob.glob(os.path.join(calibrationDir, '*.jpg'))
    if imgPathList == []:
        raise FileExistsError ('ERROR, revisar el path declarado para las imagenes RGB.')
    # Inicialización
    # nRows y nCols son el número de ESQUINAS INTERNAS del tablero
    # (no la cantidad de cuadros). Por ejemplo, un tablero de 10x7 cuadros
    # tiene 9x6 esquinas internas.
    nRows = 6
    nCols = 4

    # Criterio de parada para refinar la posición de las esquinas:
    # se detiene cuando llega a 30 iteraciones o cuando la mejora es
    # menor a 0.001 (lo que ocurra primero).
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TermCriteria_MAX_ITER, 30, 0.001)

    # Puntos "del mundo real" (coordenadas 3D del tablero, en un sistema
    # donde el propio tablero está en el plano Z=0). Se generan una sola
    # vez y se reutilizan para todas las imágenes donde sí se detecte el
    # patrón, porque el tablero físico siempre es el mismo.
    worldPtsCur = np.zeros((nRows * nCols, 3), np.float32)
    #worldPtsCur con las coordenadas reales de cada esquina
    worldPtsCur[:, :2] = np.mgrid[0:nRows, 0:nCols].T.reshape(-1, 2)


    worldPtsList = []  # Aquí se acumulan los puntos 3D de cada imagen válida
    imgPtsList = []    # Aquí se acumulan los puntos 2D (en píxeles) detectados

    # Buscar esquinas del tablero en cada imagen
    for curImgPath in imgPathList:
        imgBGR = cv.imread(curImgPath)
        # Se convierte a escala de grises porque la detección de esquinas
        # trabaja sobre intensidad, no sobre color
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)

        # Intenta encontrar las esquinas internas del tablero.
        # cornersFound es True/False según si lo logró.
        # cornersOrg son las coordenadas (aproximadas) de las esquinas.
        cornersFound, cornersOrg = cv.findChessboardCorners(
            imgGray, (nRows, nCols), None)

        if cornersFound == True:
            # Si se encontró el tablero, guardamos los puntos 3D
            # correspondientes (siempre los mismos)
            worldPtsList.append(worldPtsCur)

            # Refinamos la posición de las esquinas a nivel de subpíxel
            # para mayor precisión (ventana de búsqueda de 11x11 px)
            cornersRefined = cv.cornerSubPix(
                imgGray, cornersOrg, (11, 11), (-1, -1), termCriteria)
            imgPtsList.append(cornersRefined)

            if showPics:
                # Dibuja las esquinas detectadas sobre la imagen original
                # y la muestra en pantalla durante 500 ms
                cv.drawChessboardCorners(
                    imgBGR, (nRows, nCols), cornersRefined, cornersFound)
                cv.imshow('Chessboard', imgBGR)
                cv.waitKey(500)
    cv.destroyAllWindows()

    # Calibración
    # A partir de todos los pares de puntos (mundo 3D <-> imagen 2D)
    # OpenCV calcula:
    # - repError: error de reproyección promedio (en píxeles). Cuanto más
    #   bajo, mejor calibrada está la cámara. Un valor típico "bueno" es
    #   menor a 1 píxel.
    # - camMatrix: matriz intrínseca (distancia focal, punto principal)
    # - distCoeff: coeficientes de distorsión radial y tangencial
    #   (esto es lo que corrige el efecto "ojo de pez")
    # - rvecs, tvecs: rotación y traslación de la cámara para cada imagen
    repError, camMatrix, distCoeff, rvecs, tvecs = cv.calibrateCamera(
        worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)

    print('Camera Matrix:\n', camMatrix)
    print('Reproj Error (pixels): {:.4f}'.format(repError))

    # Guardar los parámetros de calibración (se usarán luego con video)
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'calibration.npz')
    np.savez(paramPath,
             repError=repError,
             camMatrix=camMatrix,
             distCoeff=distCoeff,
             rvecs=rvecs,
             tvecs=tvecs)

    # NOTA / CORRECCIÓN: en el script original esta función no devolvía
    # nada, pero runRemoveDistrosion() la llama esperando camMatrix y
    # distCoeff de vuelta. Se agrega este 'return' para que funcione:
    return camMatrix, distCoeff


def removeDistortion(camMatrix, distCoeff):
    """
    Aplica la corrección de distorsión (des-distorsiona) a una imagen
    de prueba usando los parámetros obtenidos en calibrate().

    Parámetros:
    - camMatrix: matriz intrínseca de la cámara (de calibrate())
    - distCoeff: coeficientes de distorsión (de calibrate())

    Muestra en pantalla, lado a lado, la imagen original (con distorsión)
    y la imagen corregida, ambas con una línea recta dibujada encima para
    ver claramente cómo cambia la curvatura antes y después.
    """
    root = os.getcwd()
    # Imagen de prueba sobre la que se aplicará la corrección
    imgPath = os.path.join(root, 'img_rgb/img_test/20260825_123300_224390.jpg')
    img = cv.imread(imgPath)
    height, width = img.shape[:2]

    # Calcula una nueva matriz de cámara "óptima" para la corrección,
    # y roi (region of interest) indica el área válida sin bordes negros
    camMatrixNew, roi = cv.getOptimalNewCameraMatrix(
        camMatrix, distCoeff, (width, height), 1, (width, height))

    # Aplica la corrección de distorsión a la imagen
    imgUndist = cv.undistort(img, camMatrix, distCoeff, None, camMatrixNew)

    # Dibuja una línea recta de referencia en ambas imágenes.
    # Si la calibración funcionó bien, en la imagen corregida la línea
    # se verá recta y alineada con los objetos reales; en la original
    # con distorsión se notará la curvatura tipo "ojo de pez".
    cv.line(img, (1769, 103), (1780, 922), (255, 255, 255), 2)
    cv.line(imgUndist, (1769, 103), (1780, 922), (255, 255, 255), 2)

    plt.figure()
    plt.subplot(121)
    plt.imshow(img)
    plt.subplot(122)
    plt.imshow(imgUndist)
    plt.show()


def runCalibration():
    """Ejecuta solo el proceso de calibración, mostrando las imágenes."""
    calibrate(showPics=True)


def runRemoveDistrosion():
    """
    Ejecuta la calibración (sin mostrar imágenes) y luego corrige la
    distorsión de la imagen de prueba usando los parámetros obtenidos.
    """
    camMatrix, distCoeff = calibrate(showPics=False)
    removeDistortion(camMatrix, distCoeff)


if __name__ == '__main__':
    #runCalibration()
    # Descomenta la siguiente línea (y comenta la de arriba) para
    # probar directamente la corrección de distorsión:
    runRemoveDistrosion()
