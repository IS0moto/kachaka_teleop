import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand


DEAD_ZONE_STICK = 0.3
DEAD_ZONE_BUTTON = 0.5
RATE_SPEED_UP = 2.0

class Teleop(Node):
    def __init__(self) -> None:
        super().__init__("teleop")

        # Set Publisher
        self._publisher = self.create_publisher(
            Twist, "/kachaka/manual_control/cmd_vel", 10
        )

        # Set Subscriber
        self._lidar_subscriber = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10
        )

        # Set Action client
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )   
        self._action_client.wait_for_server()
        self.get_logger().info('Initialized')

    def _set_joy_stick(self, msg: Joy) -> dict:

        _arrow_l = _arrow_r = _arrow_u = _arrow_d = 0

        if msg.axes[6] == 1:
            _arrow_l = 1
        elif msg.axes[6] == -1:
            _arrow_r = 1
        if msg.axes[7] == 1:
            _arrow_u = 1
        elif msg.axes[7] == -1:
            _arrow_d = 1

        _pad = {
            "L_JOY_V": msg.axes[1],
            "L_JOY_H": msg.axes[0],
            "R_JOY_V": msg.axes[4],
            "R_JOY_H": msg.axes[3],
            "L2": msg.axes[2],
            "R2": msg.axes[5],
            "ARROW_U": _arrow_u,
            "ARROW_D": _arrow_d,
            "ARROW_L": _arrow_l,
            "ARROW_R": _arrow_r,
            "BUTTON_A":msg.buttons[0],
            "BUTTON_B":msg.buttons[1],
            "BUTTON_X":msg.buttons[2],
            "BUTTON_Y":msg.buttons[3],
            "L1": msg.buttons[4],
            "R1": msg.buttons[5],
            "SELECT":msg.buttons[6],
            "START":msg.buttons[7],
            "CENTER":msg.buttons[8],
            "L3": msg.buttons[9],
            "R3": msg.buttons[10]
        }

        return _pad 

    
    def _joy_callback(self, msg: Joy) -> None:
        
        _pad = self._set_joy_stick(msg = msg)

        l_stick_v = _pad["L_JOY_V"] if abs(_pad["L_JOY_V"]) > DEAD_ZONE_STICK else 0.0
        l_stick_h = _pad["L_JOY_H"] if abs(_pad["L_JOY_H"]) > DEAD_ZONE_STICK else 0.0
        l2_button = _pad["L2"]
        

        # Set Velocity
        self._cmd_vel = Twist()
        if l2_button >= 0.0:
            self._cmd_vel.linear.x = l_stick_v
            self._cmd_vel.angular.z = l_stick_h
        else:
            self.get_logger().info('[ ] speed up')
            self._cmd_vel.linear.x = l_stick_v * RATE_SPEED_UP
            self._cmd_vel.angular.z = l_stick_h * RATE_SPEED_UP

        self._publisher.publish(self._cmd_vel)

        # Set Command 
        command = KachakaCommand()
        ## SPEAK
        if _pad["CENTER"] == 1:
            command.command_type = KachakaCommand.SPEAK_COMMAND
            command.speak_command_text = "こんにちは、カチャカです"
            goal_msg = ExecKachakaCommand.Goal()
            goal_msg.kachaka_command = command
            self._action_client.send_goal_async(goal_msg)
        

        ## Carry to Work Space
        if _pad["BUTTON_A"] == 1:
            command.command_type = KachakaCommand.MOVE_SHELF_COMMAND
            command.move_shelf_command_target_shelf_id = "S01"
            command.move_shelf_command_destination_location_id = "L02"
            goal_msg = ExecKachakaCommand.Goal()
            goal_msg.kachaka_command = command
            self._action_client.send_goal_async(goal_msg)

        ## Carry back
        if _pad["BUTTON_B"] == 1:
            command.command_type = KachakaCommand.MOVE_SHELF_COMMAND
            command.move_shelf_command_target_shelf_id = "S01"
            command.move_shelf_command_destination_location_id = "S01_home"
            goal_msg = ExecKachakaCommand.Goal()
            goal_msg.kachaka_command = command
            self._action_client.send_goal_async(goal_msg)
        
        ## Dock
        if _pad["L3"] == 1:
            command.command_type = KachakaCommand.DOCK_SHELF_COMMAND
            goal_msg = ExecKachakaCommand.Goal()
            goal_msg.kachaka_command = command
            self._action_client.send_goal_async(goal_msg)
        
        ## Undock
        if _pad["L3"] == 1:
            command.command_type = KachakaCommand.UNDOCK_SHELF_COMMAND
            goal_msg = ExecKachakaCommand.Goal()
            goal_msg.kachaka_command = command
            self._action_client.send_goal_async(goal_msg)
        


def main(args = None):
    rclpy.init(args = args)
    teleop = Teleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
