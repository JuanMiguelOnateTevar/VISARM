import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from interfaces.srv import TrackedImage
from cv_bridge import CvBridge

class CameraClientNode(Node):
    def __init__(self):
        super().__init__('node_cumni_cam')

        self.bridge = CvBridge()

        qos = QoSProfile(
            depth = 1,
            history = HistoryPolicy.KEEP_LAST,
            durability = DurabilityPolicy.VOLATILE,
            reliability = ReliabilityPolicy.BEST_EFFORT
        )

        self.client_camera = self.create_client(
            srv_name='/camera/processing',
            srv_type=TrackedImage,
            qos_profile=qos
        )

    def callback_client_camera(self) -> None:

        if not self.client_camera.service_is_ready():
            self.get_logger().error('Servicio de cámara no disponible.')
            if self.cam_response_handler is None:
                self.cam_response_handler(
                    False,
                    'Servicio de la camara no ERROR',
                    None,
                    None
                )
            return
        
        try:
            self.get_logger().info('Empezando captura y procesado.')
            future = self.client_camera.call_async()

            def when_finished(complete_request) -> None:
                try:
                    response = complete_request.result()
                    success = response.success
                    message = response.message
                    time = (response.header_capture.sec*1000) + response.header_capture.nanosec
                    img = self.bridge.imgmsg_to_cv2(response.image.data, desired_encoding='bgr8')


                    self.get_logger().info('Imagen captura y procesado.')
                except Exception as error:
                    success = False,
                    message = error,
                    time = None,
                    img = None

                if self.cam_response_handler is None:
                    self.cam_response_handler(
                        success,
                        message,
                        time,
                        img
                    )

            future.add_done_callback(when_finished)
        
        except Exception as error:
            if self.cam_response_handler is None:
                self.cam_response_handler(
                    False,
                    'Servicio de la camara no ERROR',
                    None,
                    None
                )


        