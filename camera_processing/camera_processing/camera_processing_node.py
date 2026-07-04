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
            qos_profile=qos,
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

    def capture_image(self, response):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('No se pudo inicializar la camara')
            return
        frame_processing = self.processing_image(frame_raw=frame)
        srv_img = self.bridge.cv2_to_imgmsg(frame_processing, encoding='bgr8')
        response.tracked_msg.message = 'Imagen capturada y procesada'
        response.tracked_msg.header_capture.stamp = self.get_clock().now().to_msg()
        response.tracked_msg.image = srv_img

        return response

    def processing_image(self, frame_raw):
        return frame_raw
        
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
        



