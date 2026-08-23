import rclpy
from rclpy.node import Node 
from arm.utils.grbl_serial_client import GrblSerialClient
from arm.utils.arm_kinematics import inverse_kinematics, model_joints_to_grbl_targets, build_g1_command, forward_kinematics, grbl_targets_to_model_joints
from interfaces.srv import Trigger, Change
from rclpy.action import ActionServer
from interfaces.action import Home, MoveJoints, MoveJointsCart
import time

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

        self.move_joints_server = ActionServer(
            self,
            action_name="arm/move_joints",
            action_type=MoveJoints,
            execute_callback=self.execute_move_joints_callback
            )

        self.move_joints_cart = ActionServer(
            self,
            action_name="arm/move_joints_cart",
            action_type=MoveJointsCart,
            execute_callback=self.execute_move_joints_cart_callback
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
            response.success, message = self.SerialClient.query_status()
            response.message = message.get('raw')
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
            self.get_logger().info('Homming realizado')
            result.success = True
            result.message = "Homing completed"

        except (RuntimeError, TimeoutError, ConnectionError) as error:
            feedback.stage = "error"
            goal_handle.publish_feedback(feedback)

            goal_handle.abort()
            result.success = False
            result.message = f"Homing failed: {error}"

        return result
    
    def execute_move_joints_callback(self, goal_handle) -> MoveJoints.Result:
        WORK_LIMITS = {
            "shoulder": (0.0, 68.0),
            "elbow": (0.0, 128.0),
            "base": (0.0, 228.0),
            "wrist": (0.0, 157.0),
        }

        result = MoveJoints.Result()
        feedback = MoveJoints.Feedback()

        goal = goal_handle.request

        ang_artic = {
            'base': goal.base,
            'shoulder': goal.shoulder,
            'elbow': goal.elbow,
            'wrist': goal.wrist,
            'feed_rate': goal.feed_rate,
        }

        for item, value in WORK_LIMITS.items():
            if not value[0] <= ang_artic[item] <= value[1]:
                feedback.stage = f"Articulación '{item}' ({ang_artic[item]}) fuera de rango {value}"
                goal_handle.publish_feedback(feedback)
                goal_handle.abort()
                result.success = False
                result.message = f"Articulación '{item}' ({ang_artic[item]}) fuera de rango {value}"
                return result

        try:
            feedback.stage = "starting_move"
            feedback.grbl_status = ""
            goal_handle.publish_feedback(feedback)

            self.SerialClient.send_line_command("G21")
            self.SerialClient.send_line_command("G90")

            command = (
                f"G1 "
                f"X{ang_artic['shoulder']:.3f} "
                f"Y{ang_artic['elbow']:.3f} "
                f"Z{ang_artic['base']:.3f} "
                f"B{ang_artic['wrist']:.3f} "
                f"F{ang_artic['feed_rate']:.3f}"
            )

            self.get_logger().info(f"Sending command: {command}")

            self.SerialClient.send_line_command(command)

            deadline = time.monotonic() + 500.0

            while time.monotonic() < deadline:
                success, status_dict = self.SerialClient.query_status()

                if not success:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Error querying status: {status_dict}"
                    return result

                base_state = status_dict.get("base_status", "Unknown")
                raw_status = status_dict.get("raw", str(status_dict))

                feedback.stage = base_state
                feedback.grbl_status = raw_status

                mpos = status_dict.get("MPos")

                if mpos is not None and len(mpos) >= 5:
                    feedback.shoulder = mpos[0]
                    feedback.elbow = mpos[1]
                    feedback.base = mpos[2]
                    feedback.wrist = mpos[4]

                goal_handle.publish_feedback(feedback)

                if base_state == "Idle":
                    goal_handle.succeed()
                    result.success = True
                    result.message = "El brazo ha completado su movimiento"
                    return result

                if base_state in ["Alarm", "Hold", "Door", "Sleep"]:
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Movimiento abortado. Estado GRBL: {raw_status}"
                    return result

                time.sleep(0.2)

            goal_handle.abort()
            result.success = False
            result.message = "Timeout esperando a que el brazo llegue a Idle"
            return result

        except (RuntimeError, TimeoutError, ConnectionError) as error:
            feedback.stage = "error"
            goal_handle.publish_feedback(feedback)

            goal_handle.abort()
            result.success = False
            result.message = f"Movimiento fallido: {error}"
            return result

    def execute_move_joints_cart_callback(self, goal_handle) -> MoveJointsCart.Result:
        '''
        Este metodo es la llamada de una accion:
            -Goal es la posicion cartesiana indicada la cual llegamos x,y,z(float) y la velocidad
            de movimiento feed_rate(float) 
            -Result nos indicara si se ha completo success(bool) y un mensage message(string)
            -Feedback nos manda la posición actual x,y,z(float) y su estado grbl_status(string)
        '''
        result = MoveJointsCart.Result()
        feedback = MoveJointsCart.Feedback()

        goal = goal_handle.request
        x_c, y_c, z_c, feed_rate = goal.x, goal.y, goal.z, goal.feed_rate

        joints = inverse_kinematics(x=x_c, y=y_c, z=z_c) #Cinematica inversa de cordenadas cartesianas respecto a la base convertimos a angulos para las articualciones
        grbl_targets = model_joints_to_grbl_targets(joints=joints) #Angulos o pasos para los argunos que entienda nuestro robot
        command = build_g1_command(grbl_targets=grbl_targets, feed_rate=feed_rate) #Comandos g1 que pueda enterder la placa

        self.get_logger().info(f"Sending command: {command}")
        self.SerialClient.send_line_command(command)

        deadline = time.monotonic() + 500.0

        while time.monotonic() < deadline:
            success, status_dict = self.SerialClient.query_status()
           
            if not success:
                goal_handle.abort()
                result.success = False
                result.message = f"Error querying status: {status_dict}"
                return result

            base_state = status_dict.get("base_status", "Unknown")
            raw_status = status_dict.get("raw", str(status_dict))

            feedback.stage = base_state
            feedback.grbl_status = raw_status

            mpos = status_dict.get("MPos")
            wco = status_dict.get("WCO")

            if (mpos is not None and wco is not None and len(mpos) >= 5 and len(wco) >= 5):
                joints_grbl = {
                    "shoulder": mpos[0] - wco[0],
                    "elbow": mpos[1] - wco[1],
                    "base": mpos[2] - wco[2],
                    "wrist": mpos[4] - wco[4],
                }
                
                #self.get_logger().info(f'joints_grbl: {joints_grbl.items()}')
                model_joints = grbl_targets_to_model_joints(joints=joints_grbl) #A los angulos que nos llegan del arm tenemos que meter una correcion para ser grados reales de las articulaciones
                #self.get_logger().info(f"model_joints: {model_joints['shoulder']}, {model_joints['elbow']}, {model_joints['base']}, {model_joints['wrist']}")
                dict_feedback = forward_kinematics(ang_shoulder=model_joints['shoulder'], ang_elbow=model_joints['elbow'], ang_base=model_joints['base'], ang_wrist=model_joints['wrist']) #Cinematica directa, convierte de angulos del robot a posicion cartesiana
                #self.get_logger().info(f"dict_feedback: {dict_feedback['x']}, {dict_feedback['y']}, {dict_feedback['z']}")
                feedback.x = round(dict_feedback['x'], 2)
                feedback.y = round(dict_feedback['y'], 2)
                feedback.z = round(dict_feedback['z'], 2)

                goal_handle.publish_feedback(feedback)

            if base_state == "Idle":
                goal_handle.succeed()
                result.success = True
                result.message = "El brazo ha completado su movimiento"
                return result

            if base_state in ["Alarm", "Hold", "Door", "Sleep"]:
                goal_handle.abort()
                result.success = False
                result.message = f"Movimiento abortado. Estado GRBL: {raw_status}"
                return result

            time.sleep(0.2)

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



