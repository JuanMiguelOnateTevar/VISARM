import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from interfaces.srv import Conveyor, SetLight

class ComuniEsp32(Node):
    def __init__(self):
        super().__init__('node_comuni_esp32')

        qos = QoSProfile(
            depth = 1,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.RELIABLE 
        )

        #Cliente para el servicio de las dos luminarias
        self.client_light = self.create_client(
            srv_name="/lights/set_light",
            srv_type=SetLight
        )

        #Cliente para el control de la cinta
        self.client_conveyor = self.create_client(
            srv_name="/conveyor/set_config",
            srv_type=Conveyor
        )

        # Función de la UI que se ejecutará al recibir la fotocélula
        self.photocell_ui_callback = None
        #Subcritor a topic fotocelula
        self.subcritor_photocell = self.create_subscription(
            qos_profile=qos,
            topic="/photocell/state",
            msg_type=Bool,
            callback=self.callback_suscriptor_photocell
        )

    def callback_client_light(
        self,
        light_id: int,
        enabled: bool,
    ) -> None:

        request = SetLight.Request()
        request.light_id = light_id
        request.enabled = enabled

        if not self.client_light.service_is_ready():
            if self.light_response_handler is not None:
                self.light_response_handler(
                    light_id,
                    enabled,
                    False,
                    "El servicio de luminarias no está disponible.",
                )
            return

        try:
            future = self.client_light.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()

                    status = response.success
                    message = response.message

                except Exception as error:
                    status = False
                    message = str(error)

                if self.light_response_handler is not None:
                    self.light_response_handler(
                        light_id,
                        enabled,
                        status,
                        message,
                    )
            #Cuando future tenga una respuesta ejecuta when_finished, qued aregistrado pero no se ejecuta hasta q no llega respest, no se queda esperando
            future.add_done_callback(when_finished)

        except Exception as error:
            if self.light_response_handler is not None:
                self.light_response_handler(
                    light_id,
                    enabled,
                    False,
                    str(error),
                )
        
    def callback_client_conveyor(self, start_stop: bool, direction: bool, speed: int) -> bool:

        if not self.client_conveyor.service_is_ready():
            self.get_logger().error(
                "El servicio del conveyor no está disponible."
            )
            self.conveyor_response_handler(start_stop, direction, speed, False)
            return

        request = Conveyor.Request()
        request.start_stop = start_stop
        request.direction = direction
        request.speed = speed

        try:
            future = self.client_conveyor.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()
                    status = response.status
                except Exception as error:
                    status = False

                if self.conveyor_response_handler is not None:
                    self.conveyor_response_handler(start_stop, direction, speed, status)

            future.add_done_callback(when_finished)


        except Exception as error:
            self.get_logger().error(
                f"Error enviando la petición: {error}"
            )
            self.conveyor_response_handler(start_stop, direction, speed, False)

        
    def callback_suscriptor_photocell(self, msg: Bool) -> None:
        self.get_logger().info(f"Callback ROS ejecutado: {msg.data}")

        if self.photocell_ui_callback is not None:
            print("Enviando valor a la interfaz")
            self.photocell_ui_callback(msg.data)
        else:
            print("photocell_ui_callback es None")