import rclpy
from rclpy.node import Node 
from arm.utils.grbl_serial_client import GrblSerialClient
from interfaces.srv import Trigger, Change
from rclpy.action import ActionServer
from interfaces.action import Home

class ArmNode(Node):
    def __init__(self):
        super().__init__('grbl_arm_bridge_node')

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timeout", 3.0)
        self.declare_parameter("gripper_timeout", 5.0)
        self.declare_parameter("homing_timeout", 240.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.gripper_timeout = self.get_parameter("gripper_timeout").get_parameter_value().double_value
        self.homing_timeout = self.get_parameter("homing_timeout").get_parameter_value().double_value

        self.SerialClient = GrblSerialClient(port=port, baud_rate=baud_rate, timeout=timeout)

        self.connect_service = self.create_service(
            srv_name='/arm/connect',
            srv_type=Trigger,
            callback=self.callback_connect
        )

        self.disconnect_service = self.create_service(
            srv_name="/arm/disconnect",
            srv_type=Trigger,
            callback=self.callback_disconnect
        )

        self.status_service = self.create_service(
            srv_name="/arm/status",
            srv_type=Trigger,
            callback=self.callback_status
        )

        self.gripper_service = self.create_service(
            srv_name="/arm/gripper",
            srv_type=Change,
            callback=self.callback_gripper
        )

        self.home_action_server = ActionServer(
            self,
            action_name="/arm/home",
            action_type=Home,
            execute_callback=self.execute_home_callback
        )

    def callback_connect(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            response.success, response.message = self.SerialClient.connect()
            self.get_logger().info('Conexión realizada.')
            return response
        except (TimeoutError, ConnectionError) as error:
            response.success = False
            response.message = f'Se ha producido en error con la conexion, error: {error}'
            return response
        
    def callback_disconnect(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            response.success, response.message = self.SerialClient.disconnect()
            return response
        except (RuntimeError, TimeoutError, ConnectionError) as error:
            response.success = False
            response.message = f'Se ha producido en error con la desconexion, error: {error}'
            return response
        
    def callback_status(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            response.success, response.message = self.SerialClient.query_status()
            return response
        except (TimeoutError, ConnectionError) as error:
            response.success = False
            response.message = f'Se ha producido en error al ver el estado, error: {error}'
            return response
        
    def callback_gripper(self, request: Change.Request, response: Change.Response) -> Change.Response:
        try:
            status = request.mode 
            mode = 'abierto' if status else('cerrado' if not status else 'desconocido')
            timeout = self.gripper_timeout
            if status:
                responses=self.SerialClient.send_line_command("M64 P2", timeout)
            elif not status:
                responses=self.SerialClient.send_line_command("M65 P2", timeout)

            response.status = True 
            response.message = f'Gripper {mode}.'

            return response

        except (RuntimeError, TimeoutError, ConnectionError) as error:
            response.status = False
            response.message = f'Gripper {mode} ERROR: {error}.'
            return response 

    def execute_home_callback(self, goal_handle) -> Home.Result:
        result = Home.Result()
        feedback = Home.Feedback()

        if not goal_handle.request.start:
            goal_handle.abort()
            result.success = False
            result.message = "Homing goal rejected: start is false"
            return result

        try:
            feedback.stage = "starting_homing"
            goal_handle.publish_feedback(feedback)

            self.SerialClient.send_line_command("$H")

            _, response = self.SerialClient.query_status()
            feedback.stage = f"homing_done MPos={response.get('MPos')}"
            goal_handle.publish_feedback(feedback)

            feedback.stage = "completed"
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True
            result.message = "Homing completed"

        except (RuntimeError, TimeoutError, ConnectionError) as error:
            feedback.stage = "error"
            goal_handle.publish_feedback(feedback)

            goal_handle.abort()
            result.success = False
            result.message = f"Homing failed: {error}"

        return result

    def close(self) -> None:
        try:
            self.SerialClient.disconnect()
        except Exception as error:
            self.get_logger().error(
                f"Error cerrando la conexión serie: {error}"
            )

def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node = ArmNode()  
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"ERROR --> {e}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()  



