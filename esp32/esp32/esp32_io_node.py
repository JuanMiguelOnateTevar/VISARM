import rclpy
from rclpy.node import Node
from esp32.utils.esp32_modbus_client import Maquina
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from interfaces.srv import SetLight, Conveyor

class Esp32Node(Node):
    def __init__(self):
        super().__init__("esp32_io_node")

        self.m=Maquina(ip='192.168.10.2')

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )

        #Publisher Fotocelula
        self.photocell_publisher = self.create_publisher(
            msg_type=Bool,
            qos_profile=qos,
            topic="/photocell/state"
        )
        self.last_status_fotocelula = False
        self.photocell_timer = self.create_timer(
            timer_period_sec=0.2,
            callback=self.callback_fotocelula_bridge
        )

        #Service Luminarias
        self.lights_service = self.create_service(
            srv_type=SetLight,
            srv_name="/lights/set_light",
            callback=self.callback_lights_bridge
        )

        #Service cinta
        self.cinta_service = self.create_service(
            srv_type=Conveyor,
            srv_name="/conveyor/set_config",
            callback=self.callback_conveyor_bridge
        )

    #Callback bridge fotocelula
    def callback_fotocelula_bridge(self):
        status_fotocelula = Bool()
        status_fotocelula.data = self.m.fotocelula()
        if not status_fotocelula.data == self.last_status_fotocelula:
            self.last_status_fotocelula = status_fotocelula.data
            self.photocell_publisher.publish(
                msg=status_fotocelula
            )

    #Callback bridge lights
    def callback_lights_bridge(self, request, response):

        select_light = request.light_id
        enabled = request.enabled

        match select_light:
            case 1:
                response.success = self.m.luminaria1(enabled)
                response.message = 'Luminaria 1 cambio.'
            case 2:
                response.success = self.m.luminaria2(enabled)
                response.message = 'Luminaria 2 cambio.'
            case 3:
                status1 = self.m.luminaria1(enabled)
                status2 = self.m.luminaria2(enabled)
                if status1 and status2:
                    response.success = True
                    response.message = 'Luminaria 1 y 2 cambio.'
                else:
                    response.success = False
            case _:
                response.success = False
                response.message = 'Luminaria seleccionada no existe.'

        return response
    
    def callback_conveyor_bridge(self, request, response):
        start_stop = request.start_stop
        direction = request.direction
        speed = request.speed

        # Solicitud de parada
        if not start_stop:
            status_stop = self.m.cinta_parar()

            response.status = bool(status_stop)

            # self.get_logger().info(
            #     f"Parada conveyor. Resultado: {response.status}"
            # )

            return response

        # Solicitud de marcha
        if direction:
            status_direction = self.m.cinta_adelante()
        else:
            status_direction = self.m.cinta_atras()

        if 0 <= speed <= 255:
            status_speed = self.m.set_velocidad(speed)
        else:
            self.get_logger().error(
                f"Velocidad fuera de rango: {speed}"
            )
            status_speed = False

        # Solo confirma si dirección y velocidad se aplicaron correctamente
        # response.status = bool(
        #     status_direction and status_speed
        # )
        response.status = True

        # self.get_logger().info(
        #     "Orden conveyor: "
        #     f"start_stop={start_stop}, "
        #     f"direction={direction}, "
        #     f"speed={speed}, "
        #     f"status_direction={status_direction}, "
        #     f"status_speed={status_speed}, "
        #     f"response={response.status}"
        # )

        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Esp32Node()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"ERROR --> {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()






