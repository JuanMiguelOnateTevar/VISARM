import sys
from threading import Thread
import numpy as np
import cv2

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import pyqtSignal
from frontend.ui.visarm_interfaz import Ui_MainWindow
import frontend.ui.resource

import rclpy
from rclpy.executors import MultiThreadedExecutor
from frontend.node_comuni_esp32 import ComuniEsp32
from frontend.node_comuni_cam import CameraClientNode

class VisarmWindow(QMainWindow):
    photocell_signal = pyqtSignal(bool)
    light_response_signal = pyqtSignal(
        int,   # light_id
        bool,  # enabled solicitado
        bool,  # status de la respuesta
        str,   # message
    )
    conveyor_response_signal = pyqtSignal(
        bool, #start_stop
        bool, #direction
        int, #direction
        bool #status
    )
    camera_response_signal = pyqtSignal(
        bool, #success
        str,  #message
        int,  #header_capture
        np.ndarray #image
    )

    def __init__(self) -> None:
        super().__init__()
        self.ComEsp32 = ComuniEsp32()
        self.CamClient = CameraClientNode()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # La señal Qt ejecutará photocell_received en el hilo de la interfaz
        self.photocell_signal.connect(self.photocell_received)
        # ROS emitirá la señal, no llamará directamente al widget
        self.ComEsp32.photocell_ui_callback = (
            self.photocell_signal.emit
        )
        #Actualizacion visual en Qt, aqui la dejamos la conexion preaprada y update ligt_responde solo se ejecuta cuando light_response_signal responde
        self.light_response_signal.connect(
            self.update_light_response
        )
        #ESto hace que self.ComEsp32.light_response_handler = self.light_response_signal.emit
        #Cuando tengmaos una respuesta en el nodo light_respoonse_hander ahora light_response_signal.emit este ejecutara update_light_response
        self.ComEsp32.light_response_handler = (
            self.light_response_signal.emit
        )

        self.conveyor_response_signal.connect(self.update_conveyor_response)
        self.ComEsp32.conveyor_response_handler = (self.conveyor_response_signal.emit)

        self.camera_response_signal.connect(self.update_camera_response)
        self.CamClient.cam_response_handler = (self.camera_response_signal.emit)

        self.connect_signals()

        self.status_start_stop_conveyor = False
        self.status_direction_conveyor = False
        self.status_speed_conveyor = 0

        self.ui.ButtonLaunch.clicked.connect(self.start_system)

    def photocell_received(self, detected: bool) -> None:
        if detected:
            self.ui.lineEdit_Info.setText(
                "Pieza detectada por la fotocélula."
            )
            self.ui.ButtonMarcha_Paro.setChecked(False)
            self.CamClient.callback_client_camera()

    def connect_signals(self) -> None:
        self.ui.ButtonLaunch.clicked.connect(self.start_system)
        self.ui.ButtonLuminaria1.toggled.connect(self.change_light_1)
        self.ui.ButtonLuminaria2.toggled.connect(self.change_light_2)
        self.ui.ButtonMarcha_Paro.toggled.connect(self.start_stop_conveyor)
        self.ui.ButtonSentido.toggled.connect(self.direction_conveyor)
        self.ui.speedSliderConveyor.valueChanged.connect(self.speed_conveyor)

    def start_system(self) -> None:
        print("VISARM iniciado")

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
    def update_conveyor_response(self, start_stop, direction, speed, status) -> None:
        status_start_stop = 'en marcha' if start_stop else 'parado' if not start_stop else 'ERROR'
        status_direction = 'adelante' if direction else 'atras' if not direction else 'ERROR'
        status_speed = speed if speed != 0 else 150 if speed == 0 else 'ERROR'
        if status:
            self.ui.lineEdit_Info.setText(f'Parametro de conveyo modificado. '
                f'Conveyo {status_start_stop} con dirección {status_direction} y velocidad {round(status_speed/2.55)}%')

    def update_camera_response(self, success, message, header_capture, image) -> None:
        print('----->>> Captura de imagen')
        self.ui.lineEdit_Info.setText(message)

def main(args=None) -> None:
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    window = VisarmWindow()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(window.ComEsp32)
    executor.add_node(window.CamClient)

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

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()