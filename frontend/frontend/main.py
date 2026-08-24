import sys
from threading import Thread
import numpy as np
import cv2

from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from frontend.ui.visarm_interfaz import Ui_MainWindow
import frontend.ui.resource

from pathlib import Path
import os
import datetime

import rclpy
from rclpy.executors import MultiThreadedExecutor
from frontend.node_comuni_esp32 import ComuniEsp32
from frontend.node_comuni_cam import CameraClientNode
from frontend.node_comuni_arm import ArmClientNode

class VisarmWindow(QMainWindow):
    ###ESP32###
    #FOTOCELULA#
    photocell_signal = pyqtSignal(bool)
    #LUMINARIAS#
    light_response_signal = pyqtSignal(
        int,   # light_id
        bool,  # enabled solicitado
        bool,  # status de la respuesta
        str,   # message
    )
    #CINTA#
    conveyor_response_signal = pyqtSignal(
        bool, #start_stop
        bool, #direction
        int, #direction
        bool #status
    )
    ###CAMARA###
    camera_response_signal = pyqtSignal(
        bool, #success
        str,  #message
        int,  #timestamp en milisegundos
        np.ndarray, #image Numpy o None
        float,  #cord x
        float   #cord y
    )
    ###ARM###
    #ARM_CONNECT#
    arm_connect_response_signal = pyqtSignal(
        bool, #success
        str,  #message
    )
    #ARM_DISCONNECT#
    arm_disconnect_response_signal = pyqtSignal(
        bool, #success
        str,  #message
    )
    #ARM_STATUS#
    arm_status_response_signal = pyqtSignal(
        bool, #success
        str,  #message
    )
    #ARM_HOMEandARM_JOINTS#
    arm_home_joints_response_signal = pyqtSignal(
        str, #action_name
        bool, #success
        str,  #message
    )

    z_precinta = 120.0
    feed_rate_med =150.0
    feed_rate_slow =100.0
    feed_rate_fast =200.0

    def __init__(self) -> None:
        super().__init__()
        self.ComEsp32 = ComuniEsp32()
        self.CamClient = CameraClientNode()
        self.ArmClient = ArmClientNode()


        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        ###ESP32###
        #FOTOCELULA#
        # La señal Qt ejecutará photocell_received en el hilo de la interfaz
        self.photocell_signal.connect(self.photocell_received)
        # ROS emitirá la señal, no llamará directamente al widget
        self.ComEsp32.photocell_ui_callback = (
            self.photocell_signal.emit
        )
        #LUMINARIAS#
        #Actualizacion visual en Qt, aqui la dejamos la conexion preaprada y update ligt_responde solo se ejecuta cuando light_response_signal responde
        self.light_response_signal.connect(
            self.update_light_response
        )
        #ESto hace que self.ComEsp32.light_response_handler = self.light_response_signal.emit
        #Cuando tengmaos una respuesta en el nodo light_respoonse_hander ahora light_response_signal.emit este ejecutara update_light_response
        self.ComEsp32.light_response_handler = (
            self.light_response_signal.emit
        )
        #CINTA#
        self.conveyor_response_signal.connect(self.update_conveyor_response)
        self.ComEsp32.conveyor_response_handler = (self.conveyor_response_signal.emit)
        ###CAMERA###
        self.camera_response_signal.connect(self.update_camera_response)
        self.CamClient.cam_response_handler = (self.camera_response_signal.emit)
        #Handler, los usamos para separar hilos. Los metodos o funciones de Qt se ejecutan en el hilo de Qt y los metodos o funciones de ROS se ejecutaran en el hilo de ROS corresponsiente, con esto evitamos problemas de sincronismo.
        ###ARM##
        #ARM_CONNECT#
        self.arm_connect_response_signal.connect(self.update_arm_con_response)
        self.ArmClient.arm_con_response_handler = (self.arm_connect_response_signal.emit)
        #ARM_DISCONNECT#
        self.arm_disconnect_response_signal.connect(self.update_arm_disc_response)
        self.ArmClient.arm_disc_response_handler = (self.arm_disconnect_response_signal.emit)
        #ARM_STATUS#
        self.arm_status_response_signal.connect(self.update_arm_status_response)
        self.ArmClient.arm_status_response_handler = (self.arm_status_response_signal.emit)
        #ARM_HOMEandJOINTS#
        self.arm_home_joints_response_signal.connect(self.update_arm_action_result)
        self.ArmClient.arm_action_result_handler = (self.arm_home_joints_response_signal.emit)

        self.connect_signals()

        self.status_start_stop_conveyor = False
        self.status_direction_conveyor = False
        self.status_speed_conveyor = 0

        self.ui.ButtonLaunch.setCheckable(True)
        self.ui.ButtonLaunch.toggled.connect(self.start_system)

        # Arrancamos con los botones desabilitados
        self.ui.ButtonLuminaria1.setEnabled(False)
        self.ui.ButtonLuminaria2.setEnabled(False)
        self.ui.ButtonMarcha_Paro.setEnabled(False)
        self.ui.ButtonSentido.setEnabled(False)
        self.ui.speedSliderConveyor.setEnabled(False)
        self.ui.pushButtonHome.setEnabled(False)
        self.ui.pushButtonPrePicking.setEnabled(False)
        self.ui.pushButtonOK.setEnabled(False)
        self.ui.pushButtonNOK.setEnabled(False)
        self.ui.pushButtonStatus.setEnabled(False)
        #Desabilitamos linea del path y linea info para que no puedas escribir directamente
        self.ui.lineEdit_Path.setEnabled(False)
        self.ui.lineEdit_Info.setEnabled(False)
        self.ui.checkBox_Save.setEnabled(False)

    def connect_signals(self) -> None:
        self.ui.toolButton_Path_Select.clicked.connect(self.select_path)
        self.ui.ButtonLuminaria1.toggled.connect(self.change_light_1)
        self.ui.ButtonLuminaria2.toggled.connect(self.change_light_2)
        self.ui.ButtonMarcha_Paro.toggled.connect(self.start_stop_conveyor)
        self.ui.ButtonSentido.toggled.connect(self.direction_conveyor)
        self.ui.speedSliderConveyor.valueChanged.connect(self.speed_conveyor)
        self.ui.pushButtonStatus.clicked.connect(self.arm_status)
        self.ui.pushButtonHome.clicked.connect(self.arm_home)

    def start_system(self, checked) -> None:
        if checked:
            # Activar botones
            self.ui.ButtonLuminaria1.setEnabled(True)
            self.ui.ButtonLuminaria2.setEnabled(True)
            self.ui.ButtonMarcha_Paro.setEnabled(True)
            self.ui.ButtonSentido.setEnabled(True)
            self.ui.speedSliderConveyor.setEnabled(True)
            self.ui.pushButtonHome.setEnabled(True)
            self.ui.pushButtonPrePicking.setEnabled(True)
            self.ui.pushButtonOK.setEnabled(True)
            self.ui.pushButtonNOK.setEnabled(True)
            self.ui.pushButtonStatus.setEnabled(True)

            # Conectar arm y poner home arm, encendemos la luminaria1, luminaria2 y cinta.
            self.arm_connect()
            self.ui.ButtonLuminaria1.setChecked(True)
            self.ui.ButtonLuminaria2.setChecked(True)
            self.ui.speedSliderConveyor.setValue(120)
            self.ui.ButtonSentido.setChecked(True)
            self.ui.ButtonMarcha_Paro.setChecked(True)
            
            self.ui.lineEdit_Info.setText("VISARM iniciado")

        else:
            # Desactivar botones
            self.ui.ButtonLuminaria1.setEnabled(False)
            self.ui.ButtonLuminaria2.setEnabled(False)
            self.ui.ButtonMarcha_Paro.setEnabled(False)
            self.ui.ButtonSentido.setEnabled(False)
            self.ui.speedSliderConveyor.setEnabled(False)
            self.ui.pushButtonHome.setEnabled(False)
            self.ui.pushButtonPrePicking.setEnabled(False)
            self.ui.pushButtonOK.setEnabled(False)
            self.ui.pushButtonNOK.setEnabled(False)
            self.ui.pushButtonStatus.setEnabled(False)

            # Desconectamos Arm, apagamos luminaria1, luminaria2 y cinta
            self.arm_disconnect()
            self.ui.ButtonLuminaria1.setChecked(False)
            self.ui.ButtonLuminaria2.setChecked(False)
            self.ui.speedSliderConveyor.setValue(0)
            self.ui.ButtonSentido.setChecked(False)
            self.ui.ButtonMarcha_Paro.setChecked(False)

            self.ui.lineEdit_Info.setText("VISARM parado")

    #Creación de las carpetas de imagenes con defectos NOK y sin defectos OK
    def select_path(self):
        path = QFileDialog.getExistingDirectory(self, "Seleccionar carpeta")

        if path:
            self.ui.lineEdit_Path.setText(path)
            self.path_base = Path(path)
            self.folder_ok = self.path_base / "OK"
            self.folder_nok = self.path_base / "NOK"
            self.folder_ok.mkdir(exist_ok=True)
            self.folder_nok.mkdir(exist_ok=True)
            self.ui.lineEdit_Info.setText("Path creado")
            self.ui.checkBox_Save.setEnabled(True)

    ###ESP32###
    #FOTOCELULA#
    #Funcion fotocelua ejecutada directamente en hilo Qt, tiene un puente para la camara entre hilo Qt -> hilo Ros
    def photocell_received(self, detected: bool) -> None:
        if not detected:
            return

        self.ui.lineEdit_Info.setText(
            "Pieza detectada. Esperando para capturar imagen..."
        )

        self.ui.ButtonMarcha_Paro.setChecked(False)

        #Este delay es no bloqueante
        QTimer.singleShot(
            1000,
            self.CamClient.callback_client_camera,
        )
    #LUMINARIAS#
    #Funciones luces puente entre hilo Qt -> hilo Ros
    def change_light_1(self, checked: bool) -> None:
        self.ComEsp32.callback_client_light(
            light_id=1,
            enabled=checked,
        )
    def change_light_2(self, checked: bool) -> None:
        self.ComEsp32.callback_client_light(
            light_id=2,
            enabled=checked,
        )    
    #Funcion luces ejectuada en hilo Qt, despues de recibir la respuesta de Ros
    def update_light_response(
        self,
        light_id: int,
        enabled: bool,
        status: bool,
        message: str,
    ) -> None:

        if status and not enabled:
            self.ui.lineEdit_Info.setText(
                f"Luminaria {light_id} apagada. {message}"
            )
        elif not status and enabled:
            self.ui.lineEdit_Info.setText(
                f"Luminaria {light_id} encendida. {message}"
            )
        else:
            self.ui.lineEdit_Info.setText(
                f"Error en luminaria {light_id}: {message}"
            )
    #CINTA#
    #Funciones cinta puente entre hilo Qt -> hilo Ros
    def start_stop_conveyor(self, checked: bool) -> None:
        if checked:
            self.status_start_stop_conveyor = True
            self.ComEsp32.callback_client_conveyor(start_stop=self.status_start_stop_conveyor, 
                                                   direction=self.status_direction_conveyor, 
                                                   speed=self.status_speed_conveyor)

            self.ui.lineEdit_Info.setText('Start conveyor.')
        else:
            self.status_start_stop_conveyor = False
            self.ComEsp32.callback_client_conveyor(start_stop=self.status_start_stop_conveyor, 
                                                   direction=self.status_direction_conveyor, 
                                                   speed=self.status_speed_conveyor)

            self.ui.lineEdit_Info.setText('Stop conveyor.')
    def direction_conveyor(self, checked: bool) -> None:
        if checked:
            self.status_direction_conveyor = True
            self.ComEsp32.callback_client_conveyor(start_stop=self.status_start_stop_conveyor,
                                                            direction=self.status_direction_conveyor,
                                                            speed=self.status_speed_conveyor)
        else:
            self.status_direction_conveyor = False
            self.ComEsp32.callback_client_conveyor(start_stop=self.status_start_stop_conveyor,
                                                            direction=self.status_direction_conveyor,
                                                            speed=self.status_speed_conveyor)

        self.ui.lineEdit_Info.setText('Dirección del conveyor modificada.')
    def speed_conveyor(self, value: int) -> None:
        if value < 90:
            self.ui.speedSliderConveyor.setValue(90)
            value = 90
        self.status_speed_conveyor = value
        self.ComEsp32.callback_client_conveyor(start_stop=self.status_start_stop_conveyor,
                                                        direction=self.status_direction_conveyor,
                                                        speed=self.status_speed_conveyor)
     
        self.ui.lineEdit_Info.setText('Velocidad del conveyor modificada.')
    #Funcion cinta ejectuada en hilo Qt, despues de recibir la respuesta de Ros
    def update_conveyor_response(self, start_stop, direction, speed, status) -> None:
        status_start_stop = 'en marcha' if start_stop else 'parado' if not start_stop else 'ERROR'
        status_direction = 'adelante' if direction else 'atras' if not direction else 'ERROR'
        status_speed = speed if speed != 0 else 150 if speed == 0 else 'ERROR'
        if status:
            self.ui.lineEdit_Info.setText(f'Parametro de conveyo modificado. '
                f'Conveyo {status_start_stop} con dirección {status_direction} y velocidad {round(status_speed/2.55)}%')
    #Funcion camara ejecutada directamente en hilo Qt
    def update_camera_response(
        self,
        success: bool,
        message: str,
        header_capture: int,
        image: np.ndarray,
        x: float,
        y: float,
    ) -> None:

        self.ui.lineEdit_Info.setText(message)

        if not success or image is None:
            print(f"Error recibiendo imagen: {message}")
            return

        try:
            #Mandamos orden de posicion de picking al Arm
            self.arm_joint_picking(x=x, y=y, z=self.z_precinta, feed_rate=self.feed_rate_med)
            # OpenCV entrega BGR y QImage necesita RGB
            image_rgb = cv2.cvtColor(
                image,
                cv2.COLOR_BGR2RGB,
            )

            image_rgb = np.ascontiguousarray(image_rgb)

            height, width, channels = image_rgb.shape
            bytes_per_line = width * channels

            qimage = QImage(
                image_rgb.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888,
            ).copy()

            pixmap = QPixmap.fromImage(qimage)

            # Configuración del QLabel
            self.ui.Image.setScaledContents(False)
            self.ui.Image.setAlignment(Qt.AlignCenter)

            # Tamaño disponible dentro del QLabel,
            # que debe ocupar el QGroupBox mediante un layout
            available_size = self.ui.Image.contentsRect().size()

            scaled_pixmap = pixmap.scaled(
                available_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )

            # Sustituye la imagen anterior
            self.ui.Image.setPixmap(scaled_pixmap)

            # Fuerza el repintado visual
            self.ui.Image.update()

            #Save en el path la imagen formato numpy si tenemo el Checked selecionado
            if self.ui.checkBox_Save.isChecked():
                self.save_img(message=message, image=image)

            print(
                f"Imagen mostrada: {width}x{height} → "
                f"{scaled_pixmap.width()}x{scaled_pixmap.height()}, "
                f"timestamp={header_capture}"
            )

        except Exception as error:
            print(f"Error mostrando imagen: {error}")

            self.ui.lineEdit_Info.setText(
                f"Error mostrando imagen: {error}"
            )
    #Pone nombre a nuestra imagen y lo guarda en funcion de si es OK o NOK en diferentes carpetas.
    def save_img(self, message:str, image: np.array) -> None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{timestamp}.npy"
        if message == 'OK':
            save_path = self.folder_ok / filename
        elif message == 'NOK':
            save_path = self.folder_nok / filename
        else:
            save_path = self.path_base / filename
        np.save(save_path, image)

    ###ARM###
    #ARM_CONNECT#
    #Funcion arm_connect puente entre hilo Qt -> hilo Ros
    def arm_connect(self) -> None:
        self.ArmClient.callback_client_connect()
    #Funcion arm_connect ejectuada en hilo Qt, despues de recibir la respuesta de Ros
    def update_arm_con_response(self, success: bool, message: str) -> None:
        if success:
            self.ui.lineEdit_Info.setText(message)
    #ARM_DISCONNECT#
    #Funcion arm_disconnect puente entre hilo Qt -> hilo Ros
    def arm_disconnect(self) -> None:
        self.ArmClient.callback_client_disconnect()
     #Funcion arm_disconnect ejectuada en hilo Qt, despues de recibir la respuesta de Ros
    def update_arm_disc_response(self, success: bool, message: str) -> None:
        if success:
            self.ui.lineEdit_Info.setText(message)
    #ARM_STATUS#
    #Funcion arm_disconnect puente entre hilo Qt -> hilo Ros
    def arm_status(self, checked: bool) -> None:
        self.ArmClient.callback_client_status()
     #Funcion arm_disconnect ejectuada en hilo Qt, despues de recibir la respuesta de Ros
    def update_arm_status_response(self, success: bool, message: str) -> None:
        if success:
            self.ui.lineEdit_Info.setText(message)
    #ARM_HOME#
    def arm_home(self) -> None:
        self.ArmClient.goal_home(start=True)
    #En estos metodos englobamos todos los que envian posiciones que nos es home al robot
    def arm_joint_prepicking(self, x:float, y:float, z:float, feed_rate: float) -> None:
        self.ArmClient.goal_joint_xyz(x=x, y=y, z=z, feed_rate=feed_rate)
    def arm_joint_picking(self, x:float, y:float, z:float, feed_rate: float) -> None:
        self.ArmClient.goal_joint_xyz(x=x, y=y, z=z, feed_rate=feed_rate)
    def arm_joint_ok(self, x:float, y:float, z:float, feed_rate: float) -> None:
        self.ArmClient.goal_joint_xyz(x=x, y=y, z=z, feed_rate=feed_rate)
    def arm_joint_nok(self, x:float, y:float, z:float, feed_rate: float) -> None:
        self.ArmClient.goal_joint_xyz(x=x, y=y, z=z, feed_rate=feed_rate)
    #Función para action en hilo Qt, es comun para todas las acciones de ARM
    def update_arm_action_result(self, action_name:str, success: bool, message: str) -> None:
        if action_name == 'home':
            if success:
                self.ui.lineEdit_Info.setText(f'Home realizado. {message}')
            else:
                self.ui.lineEdit_Info.setText(f'Home NO realizado. {message}')
        elif action_name == 'move_xyz':
            pass
     


def main(args=None) -> None:
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    window = VisarmWindow()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(window.ComEsp32)
    executor.add_node(window.CamClient)
    executor.add_node(window.ArmClient)

    ros_thread = Thread(
        target=executor.spin,
        daemon=True,
    )
    ros_thread.start()

    window.showMaximized()

    exit_code = app.exec_()

    executor.shutdown()
    ros_thread.join(timeout=2.0)

    window.ComEsp32.destroy_node()
    window.CamClient.destroy_node()
    window.ArmClient.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()