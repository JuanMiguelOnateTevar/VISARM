import rclpy
from rclpy.node import Node
from interfaces.srv import Trigger, Change
from interfaces.action import Home, MoveJoints, MoveJointsCart
from rclpy.action import ActionClient

class ArmClientNode(Node):

    def __init__(self):
        super().__init__('node_comuni_arm')

        self.arm_con_response_handler = None
        self.arm_disc_response_handler = None
        self.arm_status_response_handler = None
        self.arm_action_result_handler = None
        self.goal_handles = {}

        self.client_connect = self.create_client(
            srv_name='/arm/connect',
            srv_type=Trigger
        )

        self.client_disconnect = self.create_client(
            srv_name='/arm/disconnect',
            srv_type=Trigger
        )

        self.client_status = self.create_client(
            srv_name='/arm/status',
            srv_type=Trigger
        )

        self.client_action_home = ActionClient(
            self,
            action_name='/arm/home',
            action_type=Home
        )

        self.client_action_joint = ActionClient(
            self,
            action_name='arm/move_joints_cart',
            action_type=MoveJointsCart
        )

        self.client_gripper = self.create_client(
            srv_name='/arm/gripper',
            srv_type=Change
        )

    def callback_client_connect(self) -> None:

        request = Trigger.Request()

        if not self.client_connect.service_is_ready():
            if self.arm_con_response_handler is not None:
                self.arm_con_response_handler(
                    False,
                    "El servicio del robot no está disponible.",
                )
            return

        try:
            future = self.client_connect.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()

                    status = response.success
                    message = response.message

                except Exception as error:
                    status = False
                    message = str(error)

                if self.arm_con_response_handler is not None:
                    self.arm_con_response_handler(
                        status,
                        message,
                    )
            #Cuando future tenga una respuesta ejecuta when_finished, qued aregistrado pero no se ejecuta hasta q no llega respest, no se queda esperando
            future.add_done_callback(when_finished)

        except Exception as error:
            if self.arm_con_response_handler is not None:
                self.arm_con_response_handler(
                    False,
                    str(error),
                )

    def callback_client_disconnect(self) -> None:

        request = Trigger.Request()

        if not self.client_disconnect.service_is_ready():
            if self.arm_disc_response_handler is not None:
                self.arm_disc_response_handler(
                    False,
                    "El servicio del robot no está disponible.",
                )
            return

        try:
            future = self.client_disconnect.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()

                    status = response.success
                    message = response.message

                except Exception as error:
                    status = False
                    message = str(error)

                if self.arm_disc_response_handler is not None:
                    self.arm_disc_response_handler(
                        status,
                        message,
                    )
            #Cuando future tenga una respuesta ejecuta when_finished, qued aregistrado pero no se ejecuta hasta q no llega respest, no se queda esperando
            future.add_done_callback(when_finished)

        except Exception as error:
            if self.arm_disc_response_handler is not None:
                self.arm_disc_response_handler(
                    False,
                    str(error),
                )

    def callback_client_status(self) -> None:

        request = Trigger.Request()

        if not self.client_status.service_is_ready():
            if self.arm_status_response_handler is not None:
                self.arm_status_response_handler(
                    False,
                    "El servicio del robot no está disponible.",
                )
            return

        try:
            future = self.client_status.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()

                    status = response.success
                    message = response.message

                except Exception as error:
                    status = False
                    message = str(error)

                if self.arm_status_response_handler is not None:
                    self.arm_status_response_handler(
                        status,
                        message,
                    )
            #Cuando future tenga una respuesta ejecuta when_finished, qued aregistrado pero no se ejecuta hasta q no llega respest, no se queda esperando
            future.add_done_callback(when_finished)

        except Exception as error:
            if self.arm_status_response_handler is not None:
                self.arm_status_response_handler(
                    False,
                    str(error),
                )

    def goal_home(self, start: bool) -> None:

        goal = Home.Goal()
        goal.start = start

        self.client_action_home.wait_for_server()

        future = self.client_action_home.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg:
                self.feedback_callback(
                    feedback_msg,
                    action_name="home"
                )
        )

        future.add_done_callback(
            lambda future:
                self.goal_response_callback(
                    future,
                    action_name="home"
                )
        )

    def goal_joint_xyz(self, x: float, y:float, z:float, feed_rate:float) -> None:

        goal = MoveJointsCart.Goal()
        goal.x = x
        goal.y = y
        goal.z = z
        goal.feed_rate = feed_rate

        self.client_action_joint.wait_for_server()

        future = self.client_action_joint.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg:
                self.feedback_callback(
                    feedback_msg,
                    action_name="move_xyz"
                )
        )

        future.add_done_callback(
            lambda future:
                self.goal_response_callback(
                    future,
                    action_name="move_xyz"
                )
        )

    def goal_response_callback(
        self,
        future,
        action_name: str
    ) -> None:

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning(
                f"[{action_name}] Goal rechazado"
            )
            return

        self.get_logger().info(
            f"[{action_name}] Goal aceptado"
        )

        # Guardamos el handle correspondiente
        self.goal_handles[action_name] = goal_handle

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda future:
                self.result_callback(
                    future,
                    action_name=action_name
                )
        )
    def feedback_callback(
        self,
        feedback_msg,
        action_name: str
    ) -> None:

        feedback = feedback_msg.feedback

        if action_name == "home":
            self.get_logger().info(
                f"[HOME] stage={feedback.stage}"
            )
        elif action_name == "move_xyz":
            self.get_logger().info(
                f"[MOVE XYZ] "
                f"x={feedback.x:.2f}, "
                f"y={feedback.y:.2f}, "
                f"z={feedback.z:.2f}"
            )

    def result_callback(
        self,
        future,
        action_name: str
    ) -> None:

        wrapped_result = future.result()
        result = wrapped_result.result

        self.get_logger().info(
            f"[{action_name}] "
            f"success={result.success}, "
            f"message={result.message}"
        )

        if self.arm_action_result_handler is not None:
            self.arm_action_result_handler(
                action_name,
                result.success,
                result.message
            )

        self.goal_handles.pop(
            action_name,
            None
        )

    def callback_client_gripper(self, mode: bool, timeout: float) -> None:

        request = Change.Request()
        request.mode = mode
        request.timeout = timeout

        if not self.client_gripper.service_is_ready():
            if self.arm_grip_response_handler is not None:
                self.arm_grip_response_handler(
                    False,
                    "El servicio del robot no está disponible.",
                )
            return

        try:
            future = self.client_gripper.call_async(request)

            def when_finished(completed_future) -> None:
                try:
                    response = completed_future.result()

                    status = response.status
                    message = response.message

                except Exception as error:
                    status = False
                    message = str(error)

                if self.arm_grip_response_handler is not None:
                    self.arm_grip_response_handler(
                        status,
                        message,
                    )
            #Cuando future tenga una respuesta ejecuta when_finished, qued aregistrado pero no se ejecuta hasta q no llega respest, no se queda esperando
            future.add_done_callback(when_finished)

        except Exception as error:
            if self.arm_grip_response_handler is not None:
                self.arm_grip_response_handler(
                    False,
                    str(error),
                )