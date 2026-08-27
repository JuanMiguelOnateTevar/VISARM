#curso-ros2-uned-jm/mini_camera_system/mini_camera_system/camara_node.py
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image
import cv2
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from glob import glob
import time
#import logging
from interfaces.srv import TrackedImage
from camera_processing.utils.processing_img import processing_img

class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camara_node')
        self.cap = None
        self.camera_device = None
        self.bridge = CvBridge()

        self.cap, self.camera_device = (
            self.open_available_camera()
        )

        qos = QoSProfile(
            depth = 1,
            history = HistoryPolicy.KEEP_ALL,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE
        )

        if not self.cap.isOpened():
            self.get_logger().error('No se pudo inicializar la camara')
            raise RuntimeError('No se pudo inicializar la camara')

        self.image_service = self.create_service(
            srv_type=TrackedImage,
            srv_name='/camera/processing',
            callback=self.capture_image
        )

    def open_available_camera(self):
        devices = sorted(glob("/dev/video*"))

        if not devices:
            raise RuntimeError(
                "No se encontró ningún dispositivo /dev/video*"
            )

        self.get_logger().info(
            f"Dispositivos encontrados: {devices}"
        )

        for device in devices:
            self.get_logger().info(
                f"Probando cámara {device}"
            )

            cap = cv2.VideoCapture(
                device,
                cv2.CAP_V4L2,
            )

            if not cap.isOpened():
                self.get_logger().warning(
                    f"No se pudo abrir {device}"
                )
                cap.release()
                continue

            # Algunas cámaras necesitan varios intentos iniciales
            time.sleep(1)
            for _ in range(5):
                ret, frame = cap.read()

                if (
                    ret
                    and frame is not None
                    and frame.size > 0
                ):
                    self.get_logger().info(
                        f"Cámara seleccionada: {device}"
                    )
                    return cap, device

                time.sleep(0.05)

            self.get_logger().warning(
                f"{device} se abre, pero no entrega imágenes"
            )

            cap.release()

        raise RuntimeError(
            "Ningún dispositivo de vídeo permite capturar imágenes"
        )

    def capture_image(self, _request, response):

        self.get_logger().info(
            "Petición de captura recibida"
        )

        ret = False
        frame = None

        # Descartar frames antiguos del búfer
        for _ in range(5):
            ret, frame = self.cap.read()

            if not ret:
                break

        if not ret or frame is None:
            response.success = False
            response.message = "No se pudo capturar la imagen"

            self.get_logger().error(
                response.message
            )

            return response

        try:
            frame_processing, message, pose = self.processing_image(
                frame_raw=frame
            )
            image_msg = self.bridge.cv2_to_imgmsg(
                frame_processing,
                encoding="bgr8",
            )
            
            capture_time = self.get_clock().now().to_msg()

            image_msg.header.stamp = capture_time
            image_msg.header.frame_id = "camera"

            self.get_logger().info(f"Posicion del objeto {pose['x']:.2f}, {pose['y']:.2f}.")

            response.success = True
            response.message = message
            response.x = pose['x']
            response.y = pose['y']

            response.header_capture.stamp = capture_time
            response.header_capture.frame_id = "camera"
            response.image = image_msg

            self.get_logger().info(
                f"Imagen enviada: "
                f"{capture_time.sec}."
                f"{capture_time.nanosec:09d}, "
                f"valor medio={frame.mean():.2f}"
            )

        except Exception as error:
            response.success = False
            response.message = str(error)

            self.get_logger().error(
                f"Error procesando imagen: {error}"
            )

        return response

    def processing_image(self, frame_raw):
        frame_raw_proc, message, pose = processing_img(frame_raw=frame_raw)

        return frame_raw_proc, message, pose
        
    def destroy_node(self) -> None:
        if (
            self.cap is not None
            and self.cap.isOpened()
        ):
            self.cap.release()

            self.get_logger().info(
                "Cámara liberada correctamente"
            )

        super().destroy_node()
        
def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        print(f'Error al arrancar el nodo: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        



